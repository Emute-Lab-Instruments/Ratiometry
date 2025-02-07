
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


#include "drawing.h"

#define FAST_MEM __not_in_flash("mydata")

bool core1_separate_stack = true;

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library


//===================================
//        DISPLAY portal  Class
//===================================

class displayPortal {
 public:
  uint16_t fg_color = TFT_RED;
  uint16_t bg_color = TFT_BLACK;       // This is the background colour used for smoothing (anti-aliasing)

  const int bandColour[6] = {TFT_RED, TFT_GREEN,TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_PURPLE};

  float lastEndAnglesInBand[6][6] = {{0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {60,120,180,240,300,360} };
  float lastPWAnglesInBand[6][6] = {{0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {30,90,150,210,270,330} };


  uint8_t minRadius  = 40; // Outer arc radius
  uint8_t bandThickness  = 10;     // Thickness
  uint8_t bandSeparation = 5;

  bool bReDraw = false;

 void initialise(int ratios[], float pulse_width){
  // calculate initial angles for ratios based on initial pulse width value
  // default will be equal rations and PW of 0.5 but this function will allow for
  // saved setups to be recalled
   int totalBandMetry = 0;
   int offsetAngle = 0;
   float newPWAngleForRatio, thisRatioAngle ;
   for(int outputBand = 0; outputBand < 6; outputBand++){
    // first set up to calculte new angle change point
    totalBandMetry = 0;
    offsetAngle = 0;

    lastEndAnglesInBand[0][0] = 360;
    lastPWAnglesInBand[0][0] = (360 * pulse_width);

    for(int i = 0; i <= outputBand; i++) totalBandMetry += ratios[i];
    
    // now calculate the angle for each ratio and each PW point that is in each band
    for(int thisRatio = 0; thisRatio < (outputBand+1); thisRatio++){
      
      //calculate the new  ratio angle 
      thisRatioAngle = ( 360 * (ratios[thisRatio]/float(totalBandMetry))) ;

      //calculate the poisition of the end if the ratio section in the band and the PW position
      lastEndAnglesInBand[outputBand][thisRatio] = thisRatioAngle + offsetAngle;
      lastPWAnglesInBand[outputBand][thisRatio] = (thisRatioAngle * pulse_width) + offsetAngle;
      //Serial.print(thisRatioAngle);
      //Serial.print(" ");
      offsetAngle += thisRatioAngle;
    }
    //Serial.println("");
   }
   refreshDraw();
   //bReDraw = true;
 }

 void refreshDraw(){
  int stopAngle = 0;
  int startAngle = 0;
  int radius = 0;
 
  
  for(int outputBand = 0; outputBand < 6; outputBand++){
    startAngle = 0;
    radius = minRadius+((bandSeparation+bandThickness)*outputBand);
    for(int thisRatio = 0; thisRatio < (outputBand+1); thisRatio++){
      stopAngle = lastPWAnglesInBand[outputBand][thisRatio];
      tft.drawArc(120, 120, radius, radius - bandThickness, startAngle, stopAngle, bandColour[thisRatio], bg_color);
      tft.drawArc(120, 120, radius, radius - bandThickness+1, stopAngle,lastEndAnglesInBand[outputBand][thisRatio] , bg_color, bg_color);
      tft.drawArc(120, 120, radius - bandThickness + 3, radius - bandThickness, stopAngle,lastEndAnglesInBand[outputBand][thisRatio] , bandColour[thisRatio], bg_color);
      startAngle = lastEndAnglesInBand[outputBand][thisRatio];
    }
  }

  //  //draw inner band
  // for(int outputBand = 4; outputBand < 6; outputBand++){
  //   radius = minRadius+((bandSeparation+bandThickness)*outputBand);
  //   for(int thisRatio = 0; thisRatio <= outputBand; thisRatio++){
  //     stopAngle = lastEndAnglesInBand[outputBand][thisRatio];
  //     tft.drawArc(120, 120, radius - bandThickness + 3, radius - bandThickness, startAngle, stopAngle, bandColour[thisRatio], bg_color);
  //     startAngle = stopAngle;
  //   }
  // }
 }

 void updateDraw(int ratios[], float pulse_width){
  //do nothing if no changes to show
  if (!bReDraw) return;
  // simply initialise for redawr but with a blanking
//initialise( ratios,  pulse_width);

  int totalBandMetry = 0;
  int offsetAngle = 0;
  float newEndAngleForRatio, newPWAngleForRatio, startAngle, stopAngle, thisRatioAngle, radius, angleChange ;
  bool bAngleDeltaDirection = false;
  int paintColour;
  int paintColour2;


  for(int outputBand = 0; outputBand < 6; outputBand++){
    // first set up to calculte new angle change point
    totalBandMetry = 0;
    offsetAngle = 0;
    //caluctae the band radius
    radius = minRadius+((bandSeparation+bandThickness)*outputBand);

    for(int i = 0; i <= outputBand; i++) totalBandMetry += ratios[i];
    
    // now calculate the angle for each ratio and each PW point that is in each band
    for(int thisRatio = 0; thisRatio < (outputBand+1); thisRatio++){
      
      //calculate the new  ratio angle 
      thisRatioAngle = ( 360 * (ratios[thisRatio]/float(totalBandMetry))) ;
      

      //calculate the poisition of the end if the ratio section in the band and the PW position
      newEndAngleForRatio = thisRatioAngle + offsetAngle;
      newPWAngleForRatio = (thisRatioAngle * pulse_width) + offsetAngle;
      if (thisRatio == outputBand) newEndAngleForRatio = 360;

// //Draw the PulseWidth point

      // // set the direction value (we use this to set colour)
      angleChange = newPWAngleForRatio - lastPWAnglesInBand[outputBand][thisRatio];
  
      //now calcuate the start and stop angles for the change (1) means increasing (0) decreasing
      if (angleChange<0){
        // we are getting smaller
         //if (angleChange < 5) 
         stopAngle  = lastPWAnglesInBand[outputBand][thisRatio];
         //else startAngle = newEndAngleForRatio - 5;
          startAngle = newPWAngleForRatio;
         paintColour = bg_color;
        tft.drawArc(120, 120, radius, radius - bandThickness+1, startAngle, stopAngle, bg_color , bg_color);
        tft.drawArc(120, 120, radius - bandThickness + 3, radius - bandThickness, startAngle, stopAngle, bandColour[thisRatio], bg_color);
      } else {
        //we are getting bigger
        stopAngle   = newPWAngleForRatio;
        //if (angleChange > 5) 
        startAngle  = lastPWAnglesInBand[outputBand][thisRatio];
        //else startAngle = newEndAngleForRatio - 5;        
        paintColour = bandColour[thisRatio] ;
         tft.drawArc(120, 120, radius, radius - bandThickness, startAngle, stopAngle, paintColour , bg_color);
      }

      if ((thisRatio<6)){ // counter offet iissue
        //tft.drawArc(120, 120, radius, radius - bandThickness+3, startAngle, stopAngle, paintColour, bg_color);
        lastPWAnglesInBand[outputBand][thisRatio] = newPWAngleForRatio;
      }

      // Draw the lower ring point of the ratio section
      // set the direction value (we use this to set colour)
      bAngleDeltaDirection = (lastEndAnglesInBand[outputBand][thisRatio] >= newEndAngleForRatio);
      angleChange = newEndAngleForRatio - lastEndAnglesInBand[outputBand][thisRatio];

      if (angleChange<0){
        // we are getting smaller
         if (angleChange < 5) stopAngle = lastEndAnglesInBand[outputBand][thisRatio];
         else stopAngle = newEndAngleForRatio - 5;
         startAngle = newEndAngleForRatio;
         paintColour = bandColour[thisRatio+1];
         paintColour2 = bandColour[thisRatio+1];
         tft.drawArc(120, 120, radius, radius - bandThickness, startAngle, stopAngle, paintColour , bg_color);
      } else {
        //we are getting bigger
        stopAngle = newEndAngleForRatio;
        if (angleChange > 5) startAngle = lastEndAnglesInBand[outputBand][thisRatio];
        else startAngle = newEndAngleForRatio - 5;        
        paintColour = bandColour[thisRatio];
        paintColour2 = TFT_BLACK; //bandColour[thisRatio];
        tft.drawArc(120, 120, radius, radius - bandThickness+1, startAngle, stopAngle, paintColour2 , bg_color);
        tft.drawArc(120, 120, radius - bandThickness + 3, radius - bandThickness, startAngle, stopAngle, paintColour, bg_color);
      }

      if ((thisRatio<6)){ // counter offet iissue
      //  tft.drawArc(120, 120, radius, radius - bandThickness+1, startAngle, stopAngle, paintColour2 , bg_color);
      //  tft.drawArc(120, 120, radius - bandThickness + 3, radius - bandThickness, startAngle, stopAngle, paintColour, bg_color);


      //  //and draw the bity above the band
      //  if (angleChange<0){
      //   // we are getting smaller so draw the next ratio colour
      //   tft.drawArc(120, 120, radius, radius - bandThickness+2, startAngle, stopAngle,paintColour , bg_color);
      //  }else{
      //   //we are getting bigger so use bg_colour
      //   tft.drawArc(120, 120, radius, radius - bandThickness+2, startAngle, stopAngle, bg_color , bg_color);
      //  }
       lastEndAnglesInBand[outputBand][thisRatio] = newEndAngleForRatio;
      }

      

      
       
      
      
      offsetAngle += thisRatioAngle;

    }

  }
  bReDraw = false;
  
  }

  void setReDraw(){bReDraw = true;}

};
//        EO DISPLAY portal Class
//===================================

displayPortal display;

#define SCREEN_WIDTH tft.width()    //
#define SCREEN_HEIGHT tft.height()  // Taille de l'écran

#define ENCODER1_A_PIN 9
#define ENCODER1_B_PIN 14
//#define ENCODER1_SWITCH 8

#define ENCODER2_A_PIN 16
#define ENCODER2_B_PIN 21
//#define ENCODER2_SWITCH 15

#define ENCODER3_A_PIN 24
#define ENCODER3_B_PIN 25
//#define ENCODER3_SWITCH 23

static uint8_t encoderPins[3][2] = { {ENCODER1_A_PIN,ENCODER1_B_PIN}, {ENCODER2_A_PIN,ENCODER2_B_PIN}, {ENCODER3_A_PIN,ENCODER3_B_PIN}};

namespace controls {
  static int encoderValues[6] = {5,5,5,5,5,5};
  static bool encoderSwitches[6] = {0,0,0,0,0,0};
  static float pulse_width = 0.5;
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

bool __not_in_flash_func(displayProcessor)(__unused struct repeating_timer *t) {
  display.updateDraw(controls::encoderValues, controls::pulse_width);
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

// static uint8_t encCode[3] = {0,0,0};
// static uint16_t encStore[3] = {0,0,0};

// void processEncoder(int thisEncoder){
//   Serial.println("cb");
//   int change = read_rotary(encCode[thisEncoder], encStore[thisEncoder], encoderPins[thisEncoder][0], encoderPins[thisEncoder][1]);
//   //if (change == 0) return;
//   controls::encoderValues[thisEncoder] += change;
//   if (controls::encoderValues[thisEncoder]<1) controls::encoderValues[thisEncoder]=1;
//   Serial.println(controls::encoderValues[thisEncoder]);  
//   // trigger a redraw for later
//   display.setReDraw();
// }

// void encoder1_callback() {processEncoder(0);}
// void encoder2_callback() {processEncoder(1);}
// void encoder3_callback() {processEncoder(2);}

void encoder1_callback() {
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  //if (change == 0) return;
  controls::encoderValues[0] += change;
  if (controls::encoderValues[0]<1) controls::encoderValues[0]=1;
  Serial.println(controls::encoderValues[0]);  
  // trigger a redraw for later
  display.setReDraw();
}

void encoder2_callback() {
  //int thisEncoderID = 1;
  Serial.println("cb");
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  //if (change == 0) return;
  controls::encoderValues[1] += change;
  if (controls::encoderValues[1]<1) controls::encoderValues[1]=1;
  Serial.println(controls::encoderValues[1]);  
  // trigger a redraw for later
  display.setReDraw();
}

void encoder3_callback() {
  int thisEncoderID = 2;
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  if (change == 0) return;
  controls::encoderValues[thisEncoderID] += change;
  if (controls::encoderValues[thisEncoderID]<1) controls::encoderValues[thisEncoderID]=1;
  Serial.println(controls::encoderValues[thisEncoderID]);  
  // trigger a redraw for later
  display.setReDraw();
}

// void encoder1_switch_callback() {
//   controls::encoderSwitches[0] = digitalRead(ENCODER1_SWITCH);  
// }

// void encoder2_switch_callback() {
//   controls::encoderSwitches[1] = digitalRead(ENCODER2_SWITCH);  
// }

// void encoder3_switch_callback() {
//   controls::encoderSwitches[2] = digitalRead(ENCODER3_SWITCH);  
// }


struct repeating_timer timerAdcProcessor;
struct repeating_timer timerDisplayProcessor;


void setup() {
 while (!Serial) {
        // wait for Serial to become active
    }

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(BG_COLOUR);
  tft.setFreeFont(&FreeSans18pt7b);

  // for(size_t i=0; i < 4; i++) {
  //   adcRanges[i] = adcMaxs[i] - adcMins[i];
  // }

#define LED_PIN 22

  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  // //ADCs
  // const size_t filterSize=5;
  // for(auto &filter: adcFilters) {
  //   filter.init(filterSize);
  // }
  // setup_adcs();

  //Encoders
  // for (int i = 0; i < 6; i++){
  //   for (int n = 0; n < 2; n++) pinMode(encoderPins[i][n], INPUT_PULLUP); 
  //   }
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
//  pinMode(ENCODER1_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  //pinMode(ENCODER2_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
  //pinMode(ENCODER3_SWITCH, INPUT_PULLUP);
  

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), encoder1_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), encoder2_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A_PIN), encoder3_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_B_PIN), encoder3_callback, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(ENCODER1_SWITCH), encoder1_switch_callback,
  //                   CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER2_SWITCH), encoder2_switch_callback,
  //                   CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER3_SWITCH), encoder3_switch_callback,
  //                   CHANGE);


  add_repeating_timer_ms(5, adcProcessor, NULL, &timerAdcProcessor);
  add_repeating_timer_ms(50, displayProcessor, NULL, &timerDisplayProcessor);

  //display intial ratios TODO load last values?
  display.initialise(controls::encoderValues, controls::pulse_width);
}


int count=0;
void loop() {
  __wfi();
}


void setup1() {
}

void loop1() {
  __wfi();
}
