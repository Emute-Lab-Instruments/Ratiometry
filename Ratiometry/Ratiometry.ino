
#include <optional>

#include <TFT_eSPI.h>

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

#include "MedianFilter.h"
#include "MAFilter.h"

#include "ratios.hpp"


#include "drawing.h"

#define FAST_MEM __not_in_flash("mydata")

bool core1_separate_stack = true;

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

int __not_in_flash("mydata") oscTypeBank0=0;
int __not_in_flash("mydata") oscTypeBank1=0;
int __not_in_flash("mydata") oscTypeBank2=0;


class displayPortal {
public:
};

displayPortal display;


#define SCREEN_WIDTH tft.width()    //
#define SCREEN_HEIGHT tft.height()  // Taille de l'écran

#define ENCODER1_A_PIN 0
#define ENCODER1_B_PIN 2
#define ENCODER1_SWITCH 8

#define ENCODER2_A_PIN 3
#define ENCODER2_B_PIN 5
#define ENCODER2_SWITCH 15

#define ENCODER3_A_PIN 6
#define ENCODER3_B_PIN 7
#define ENCODER3_SWITCH 23

#define ENCODER4_A_PIN 8
#define ENCODER4_B_PIN 9
#define ENCODER4_SWITCH 20

#define PULSEOUT0_PIN 14
#define PULSEOUT1_PIN 15
#define PULSEOUT2_PIN 16
#define PULSEOUT3_PIN 17


namespace controls {
  static int encoderValues[4] = {0,0,0,0};
  static bool encoderSwitches[4] = {0,0,0,0};
};

MovingAverageFilter<float> adcFilters[4];
static float __not_in_flash("mydata") controlValues[4] = {0,0,0,0};


static uint16_t __not_in_flash("mydata") capture_buf[16] __attribute__((aligned(2048)));

static FAST_MEM std::array<size_t,4> adcMins{22,22,22,22};
static FAST_MEM std::array<size_t,4> adcMaxs{4085,4085,4085,4085};
std::array<size_t,4> adcRanges;

void setup_adcs() {
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_gpio_init(29);
  adc_set_round_robin(15);
  adc_fifo_setup(
    true,   // Write each completed conversion to the sample FIFO
    true,   // Enable DMA data request (DREQ)
    1,      // DREQ (and IRQ) asserted when at least 1 sample present
    false,  // We won't see the ERR bit because of 8 bit reads; disable.
    false   // Shift each sample to 8 bits when pushing to FIFO
  );

  // Divisor of 0 -> full speed. Free-running capture with the divider is
  // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
  // cycles (div not necessarily an integer). Each conversion takes 96
  // cycles, so in general you want a divider of 0 (hold down the button
  // continuously) or > 95 (take samples less frequently than 96 cycle
  // intervals). This is all timed by the 48 MHz ADC clock.
  adc_set_clkdiv(96 * 128);

  // printf("Arming DMA\n");
  // sleep_ms(1);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_ring(&cfg, true, 3);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,    // dst
                        &adc_hw->fifo,  // src
                        -1,             // transfer count
                        true            // start immediately
  );

  // printf("Starting capture\n");
  adc_select_input(0);
  adc_run(true);
}


inline float __not_in_flash_func(adcMap)(const size_t adcIndex) {
  return (controlValues[adcIndex] - adcMins[adcIndex]) / adcRanges[adcIndex];
}


bool __not_in_flash_func(adcProcessor)(__unused struct repeating_timer *t) {
  controlValues[0] = adcFilters[0].process(capture_buf[0]);
  controlValues[1] = adcFilters[1].process(capture_buf[1]);
  controlValues[2] = adcFilters[2].process(capture_buf[2]);
  controlValues[3] = adcFilters[3].process(capture_buf[3]);
  return true;
}
bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  // tft.fillScreen(TFT_BLACK);
  // tft.setCursor(120,120);
  // tft.setTextColor(TFT_WHITE);
  // tft.println(controls::encoderValues[0]);

  // tft.setCursor(120,150);
  // tft.println(controlValues[0]);
  return true;
}


int8_t read_rotary(uint8_t &prevNextCode, uint16_t &store, int a_pin, int b_pin) {
  static int8_t rot_enc_table[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

  prevNextCode <<= 2;
  if (digitalRead(b_pin)) prevNextCode |= 0x02;
  if (digitalRead(a_pin)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;
  // Serial.println(prevNextCode);

  // If valid then store as 16 bit data.
  if (rot_enc_table[prevNextCode]) {
    store <<= 4;
    store |= prevNextCode;
    if ((store & 0xff) == 0x2b) return -1;
    if ((store & 0xff) == 0x17) return 1;
  }
  return 0;
}

static uint8_t enc1Code = 0;
static uint16_t enc1Store = 0;
static uint8_t enc2Code = 0;
static uint16_t enc2Store = 0;
static uint8_t enc3Code = 0;
static uint16_t enc3Store = 0;
static uint8_t enc4Code = 0;
static uint16_t enc4Store = 0;

int maxOscBankType = 8;

void updateOscBank(int &currOscBank, int change) {
  int newOscTypeBank = currOscBank + change;
  //clip
  newOscTypeBank = max(0, newOscTypeBank);
  newOscTypeBank = min(maxOscBankType, newOscTypeBank);
  if (newOscTypeBank != currOscBank) {
    //send
    currOscBank = newOscTypeBank;
    Serial.println(currOscBank);  
  } 

}


double ratioMinMax(double val) {
  val = std::max(val, ratioseq::RATIOMIN);
  val = std::min(val, ratioseq::RATIOMAX);
  return val;
}


void encoder1_callback() {
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  controls::encoderValues[0] += change;
  controls::encoderValues[0] = ratioMinMax(controls::encoderValues[0]);
  Serial.println(controls::encoderValues[0]);  
  ratiodata::ratios[0] = controls::encoderValues[0];
}

void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  controls::encoderValues[1] += change;
  controls::encoderValues[1] = ratioMinMax(controls::encoderValues[1]);
  Serial.println(controls::encoderValues[1]);  
  ratiodata::ratios[1] = controls::encoderValues[1];
}

void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
  controls::encoderValues[2] += change;
  controls::encoderValues[2] = ratioMinMax(controls::encoderValues[2]);
  Serial.println(controls::encoderValues[2]);  
  ratiodata::ratios[2] = controls::encoderValues[2];
}

void encoder4_callback() {
  int change = read_rotary(enc4Code, enc4Store, ENCODER4_A_PIN, ENCODER4_B_PIN);
  controls::encoderValues[3] += change;
  controls::encoderValues[3] = ratioMinMax(controls::encoderValues[3]);
  Serial.println(controls::encoderValues[3]);  
  ratiodata::ratios[3] = controls::encoderValues[3];
}

void encoder1_switch_callback() {
  controls::encoderSwitches[0] = digitalRead(ENCODER1_SWITCH);  
}

void encoder2_switch_callback() {
  controls::encoderSwitches[1] = digitalRead(ENCODER2_SWITCH);  
}

void encoder3_switch_callback() {
  controls::encoderSwitches[2] = digitalRead(ENCODER3_SWITCH);  
}
void encoder4_switch_callback() {
  controls::encoderSwitches[3] = digitalRead(ENCODER4_SWITCH);  
}


struct repeating_timer timerAdcProcessor;
struct repeating_timer timerDSPLoop;

bool __not_in_flash_func(dspLoop)(__unused struct repeating_timer *t) {
  ratiodata::phase += 0.002;
  if (ratiodata::phase > 1.0) {
    ratiodata::phase -= 1.0;
  }

  digitalWrite(PULSEOUT0_PIN, ratiodata::phase > 0.5);

  auto ratios1 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1]});
  bool seq1 = ratioseq::rpulse(ratios1, ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT1_PIN, seq1);

  auto ratios2 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2]});
  bool seq2 = ratioseq::rpulse(ratios2, ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT2_PIN, seq2);

  auto ratios3 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3]});
  bool seq3 = ratioseq::rpulse(ratios3, ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT3_PIN, seq3);

  return true;
}

void setup() {

  Serial.begin();
  // tft.begin();
  // tft.setRotation(3);
  // tft.fillScreen(ELI_BLUE);
  // tft.setFreeFont(&FreeSans18pt7b);

  ratiodata::ratios = {4,1,4,3};
  ratiodata::phase=0;
  ratiodata::pulseWidth=0.5;
  ratiodata::sampleRate = 1000.0;

  for(size_t i=0; i < 4; i++) {
    adcRanges[i] = adcMaxs[i] - adcMins[i];
  }

#define LED_PIN 22

  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  //ADCs
  const size_t filterSize=5;
  for(auto &filter: adcFilters) {
    filter.init(filterSize);
  }
  setup_adcs();

  //Encoders
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  // pinMode(ENCODER1_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  // pinMode(ENCODER2_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
  // pinMode(ENCODER3_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER4_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER4_B_PIN, INPUT_PULLUP);
  
  pinMode(PULSEOUT0_PIN, OUTPUT);
  pinMode(PULSEOUT1_PIN, OUTPUT);
  pinMode(PULSEOUT2_PIN, OUTPUT);
  pinMode(PULSEOUT3_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), encoder1_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), encoder2_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A_PIN), encoder3_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_B_PIN), encoder3_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_A_PIN), encoder4_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_B_PIN), encoder4_callback,
                    CHANGE);

  // attachInterrupt(digitalPinToInterrupt(ENCODER1_SWITCH), encoder1_switch_callback,
  //                   CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER2_SWITCH), encoder2_switch_callback,
  //                   CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER3_SWITCH), encoder3_switch_callback,
  //                   CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER4_SWITCH), encoder3_switch_callback,
  //                   CHANGE);


  add_repeating_timer_ms(5, adcProcessor, NULL, &timerAdcProcessor);
  // add_repeating_timer_ms(-1/ratiodata::sampleRate, dspLoop, NULL, &timerDSPLoop);
  add_repeating_timer_ms(-2, dspLoop, NULL, &timerDSPLoop);

}


int count=0;
void loop() {
  Serial.println("loop");
  delay(1000);
}


void setup1() {
}

void loop1() {
  __wfi();
}
