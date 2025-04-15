
#include <optional>

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include <Adafruit_NeoPixel.h>

#include "ratios.hpp"
#include "EncoderEventModels.h"
#include "DisplayPortals.h"
#include "PinConfigs.h"

#define FAST_MEM __not_in_flash("mydata")

bool core1_separate_stack = true;

//objects for the Encoder Event Models
EncoderEventModel* currentModel;      //pointer for the currently a ctive model
EncoderEventModel ratioEvents =  EncoderEventModel(); //6 Ratio event workspaces when declared!
//EncoderEventModel cvEvents = (EncoderEventModel) OtherEventModel();    //6 CV output event workspaces
//EncoderEventModel cvEvents = (EncoderEventModel) MutesEventModel();    //Mutes event workspaces ** not coed **

//decalre worksoacecounters
int currentRatioModel = 0;
int currentCVModel = 0;

//neo pixels declarations
#define PIN        9 // 
#define NUMPIXELS  6 // 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t pixelBandColours[6] = {pixels.Color(0,0,255), pixels.Color(255,255,0), pixels.Color(0,255,0), pixels.Color(255,255,255),pixels.Color(230,80,0), pixels.Color(255,9,0) }; 


DisplayPortal display;


namespace controls {
 // static int encoderValues[6] = {4,5,4,3,6,4};;
 // static bool encoderSwitches[6] = {0,0,0,0,0,0};
 // static float pulse_width = 0.5;
  //static float phaseRate = 0.001;
};

namespace outputs {
  static bool triggers[6] = {0,0,0,0,0,0};
  static int currentBeat[6] = {0,0,0,0,0,0};
};

namespace switches {
  static double t_trigger[8] = {0,0,0,0,0,0,0};
  static bool b_state[8] = {1,1,1,1,1,1,1,1};
  static int event_state[8] = {0,0,0,0,0,0,0};
};

bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  // tft.fillScreen(TFT_BLACK);
  // tft.setCursor(120,120);
  // tft.setTextColor(TFT_WHITE);
  // tft.println(controls::encoderValues[0]);

  // tft.setCursor(120,150);
  // tft.println(controlValues[0]);
  //display.update(controls::encoderValues, controls::pulse_width);
  display.drawTimer(ratiodata::phase);
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
static uint8_t enc5Code = 0;
static uint16_t enc5Store = 0;
static uint8_t enc6Code = 0;
static uint16_t enc6Store = 0;
static uint8_t enc7Code = 0;
static uint16_t enc7Store = 0;
static uint8_t enc8Code = 0;
static uint16_t enc8Store = 0;

//blue
void encoder1_callback() {  
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  currentModel->changeEncoder1(change);
  display.setReDraw();
}
//red
void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  currentModel->changeEncoder6(change);
  display.setReDraw();
}

//orange
void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
   currentModel->changeEncoder5(change);
  display.setReDraw();
}
//white
void encoder4_callback() {
  int change = read_rotary(enc4Code, enc4Store, ENCODER4_A_PIN, ENCODER4_B_PIN);
  currentModel->changeEncoder4(change);
  display.setReDraw();
}

//green
void encoder5_callback() {
  int change = read_rotary(enc5Code, enc5Store, ENCODER5_A_PIN, ENCODER5_B_PIN);
  currentModel->changeEncoder3(change);
  display.setReDraw();
}

//yelloie
void encoder6_callback() {
  int change = read_rotary(enc6Code, enc6Store, ENCODER6_A_PIN, ENCODER6_B_PIN);
  currentModel->changeEncoder2(change);
  display.setReDraw();
}

//left - pulse width
void encoder7_callback() {
  int change = read_rotary(enc7Code, enc7Store, ENCODER7_A_PIN, ENCODER7_B_PIN);
  currentModel->changeEncoder7(change);
  display.setReDraw();
}

//Right - phaseRate
void encoder8_callback() {
  int change = read_rotary(enc8Code, enc8Store, ENCODER8_A_PIN, ENCODER8_B_PIN);
  currentModel->changeEncoder8( change);
//  display.setReDraw(); No dosplay up date for speed changes ** might need to check on Model Mode later in dev if the ENC gets used**
}

void encoder1_switch_callback() {
  bool bNow = digitalRead(ENCODER1_SWITCH); 
  float t_since = millis() - switches::t_trigger[0];
  
  if ( t_since < 30) return;

  //Serial.println(bNow);
  if ((bNow == 0) && (switches::event_state[0] == 0)) {
    switches::t_trigger[0] = millis(); 
    switches::event_state[0] = 1;
  }
  if ((switches::b_state[0] == 0) && (bNow == 1)&& (switches::event_state[0] == 1)) {
    if (t_since>300) switches::event_state[0] = 3;
    else switches::event_state[0] = 2;
  }
  switches::b_state[0] = bNow;
}




void encoder2_switch_callback() {
 // controls::encoderSwitches[1] = digitalRead(ENCODER2_SWITCH);  
}

void encoder3_switch_callback() {
 // controls::encoderSwitches[2] = digitalRead(ENCODER3_SWITCH);  
}

double ratioMinMax(double val) {
  val = std::max(val, ratioseq::RATIOMIN);
  val = std::min(val, ratioseq::RATIOMAX);
  return val;
}

struct repeating_timer timerDisplayProcessor;
struct repeating_timer timerPixelDisplayProcessor;
struct repeating_timer timerDSPLoop;
struct repeating_timer timerSwitchEventsProcessor;

bool __not_in_flash_func(processSwitchEvents)(__unused struct repeating_timer *t) {
  // check if an event has been logged against a switch and then do someting useful
  //Serial.println("sw");
  for(int i = 0; i < 8; i++){
    switch(switches::event_state[i]){
      case 0:
        break;
      case 1:
        //Serial.println( String(i) + " down");
        // if ((millis()-switches::t_trigger[i])>600) {
        //   Serial.println( String(i) + "very long");
        //   switches::event_state[i]=4;
        // }
        break;
      case 2:
        Serial.println( String(i) + " short");
        switches::event_state[i]=0;
        break;
      case 3:
        Serial.println( String(i) + " long");
        switches::event_state[i]=0;
        break;
      case 4:
        
        break;     
      default:
        break;
    }
    
  }
  return true;
}

bool __not_in_flash_func(dspLoop)(__unused struct repeating_timer *t) {
  ratiodata::phase += *ratiodata::phaseRate; //0.002;
  if (ratiodata::phase > 1.0) {
    ratiodata::phase -= 1.0;
  }
  if (ratiodata::phase <0) {
    ratiodata::phase += 1.0;
  }

  digitalWrite(PULSEOUT0_PIN, ratiodata::phase < *ratiodata::pulseWidth);
  outputs::triggers[5] = ratiodata::phase < *ratiodata::pulseWidth; 
  outputs::currentBeat[5] = 0;

  auto ratios1 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1]});
  int seq1 = ratioseq::rpulse(ratios1, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT1_PIN, seq1/10);
  outputs::triggers[4] = seq1/10;
  outputs::currentBeat[4] = seq1 % 10;

  auto ratios2 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2]});
  int seq2 = ratioseq::rpulse(ratios2, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT2_PIN, seq2/10);
  outputs::triggers[3] = seq2/10;
  outputs::currentBeat[3] = seq2 % 10;

  auto ratios3 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3]});
  int seq3 = ratioseq::rpulse(ratios3, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT3_PIN, seq3/10);
  outputs::triggers[2] = seq3/10;
  outputs::currentBeat[2] = seq3 % 10; 

  auto ratios4 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3],ratiodata::ratios[4]});
  int seq4 = ratioseq::rpulse(ratios4, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT4_PIN, seq4/10);
  outputs::triggers[1] = seq4/10;
  outputs::currentBeat[1] = seq4 % 10; 

  auto ratios5 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3],ratiodata::ratios[4],ratiodata::ratios[5]});
  int seq5 = ratioseq::rpulse(ratios5, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT5_PIN, seq5/10);
  outputs::triggers[0] = seq5/10;
  outputs::currentBeat[0] = seq5 % 10; 

  return true;
}

bool __not_in_flash_func (updateNeoPixels)(__unused struct repeating_timer *t){
 for(int i = 0; i < NUMPIXELS; i++){
   pixels.setPixelColor(i, pixelBandColours[outputs::currentBeat[i]]* outputs::triggers[i]);
 }
 pixels.show();
 return true;
}

//this loads default values - later load from storage
void loadRatioModels(){
  double values[8] = {4,5,4,3,6,4, 0.9, 0.002};
  double maximums[8] = {32,32,32,32,32,32, 0.95, 0.004};
  double minimums[8] = {1,1,1,1,1,1, 0.05, -0.004};
  double steps[8] = {1,1,1,1,1,1, 0.05, 0.00005};
  ratioEvents.setEncoderValues(values);
  ratioEvents.setEncoderMaximums(maximums);
  ratioEvents.setEncoderMinimums(minimums);
  ratioEvents.setEncoderSteps(steps);
}

void setup() {


  // tft.begin();
  // tft.setRotation(3);
  // tft.fillScreen(ELI_BLUE);
  // tft.setFreeFont(&FreeSans18pt7b);

  // ratiodata::ratios = {4,5,4,3,6,4};
  // ratiodata::phase=0;
  // ratiodata::pulseWidth=0.5;
  // ratiodata::sampleRate = 1000.0;
  // ratiodata::phaseRate = 0.001;

  //loadf ratio workspaces
  loadRatioModels();

  //point current model to first workspace
  currentModel = &ratioEvents;
  ratiodata::ratios = currentModel->encoderValues;
  ratiodata::phase=0;
  ratiodata::pulseWidth=&currentModel->encoderValues[6];
  ratiodata::sampleRate = 1000.0;
  ratiodata::phaseRate = &currentModel->encoderValues[7]; //this gioves each workspace its oiwn phase rate - redefine to keep a fixed



  tft.init();
  tft.initDMA();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(2);
  tft.startWrite();
  // Create the 2 sprites, each is half the size of the screen
  sprPtr[0] = (uint16_t*)spr[0].createSprite(SCREEN_HEIGHT, SCREEN_HEIGHT / 2);
  sprPtr[1] = (uint16_t*)spr[1].createSprite(SCREEN_WIDTH, SCREEN_WIDTH / 2);

  // Move the sprite 1 coordinate datum upwards half the screen height
  // so from coordinate point of view it occupies the bottom of screen
  spr[1].setViewport(0, -SCREEN_HEIGHT / 2, SCREEN_WIDTH, SCREEN_HEIGHT);

  // Define text datum for each Sprite
  spr[0].setTextDatum(MC_DATUM);
  spr[1].setTextDatum(MC_DATUM);

  //Encoders
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER4_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER4_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER4_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER5_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER5_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER5_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER6_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER6_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER6_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER7_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER7_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER7_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER8_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER8_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER8_SWITCH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), encoder1_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), encoder2_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A_PIN), encoder3_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_B_PIN), encoder3_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_A_PIN), encoder4_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_B_PIN), encoder4_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER5_A_PIN), encoder5_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER5_B_PIN), encoder5_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER6_A_PIN), encoder6_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER6_B_PIN), encoder6_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER7_A_PIN), encoder7_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER7_B_PIN), encoder7_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER8_A_PIN), encoder8_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER8_B_PIN), encoder8_callback,CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_SWITCH), encoder1_switch_callback,CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER2_SWITCH), encoder2_switch_callback,
  //                   CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER3_SWITCH), encoder3_switch_callback,
  //                   CHANGE);

  pinMode(PULSEOUT0_PIN, OUTPUT);
  pinMode(PULSEOUT1_PIN, OUTPUT);
  pinMode(PULSEOUT2_PIN, OUTPUT);
  pinMode(PULSEOUT3_PIN, OUTPUT);
  pinMode(PULSEOUT4_PIN, OUTPUT);
  pinMode(PULSEOUT5_PIN, OUTPUT);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.setBrightness(10);

  add_repeating_timer_ms(30, displayUpdate, NULL, &timerDisplayProcessor);
  add_repeating_timer_ms(30, updateNeoPixels, NULL, &timerPixelDisplayProcessor);
  add_repeating_timer_ms(40, processSwitchEvents, NULL, &timerSwitchEventsProcessor);


  //display intial ratios
  //display.update(controls::encoderValues, controls::pulse_width);

  add_repeating_timer_ms(-2, dspLoop, NULL, &timerDSPLoop);
  display.setReDraw();
}


int count=0;
void loop() {
  //__wfi();
  //display.update(controls::encoderValues, controls::pulse_width);
  display.update(ratiodata::ratios, *ratiodata::pulseWidth);

  delay(20);
}


void setup1() {
}

void loop1() {
  __wfi();
}
