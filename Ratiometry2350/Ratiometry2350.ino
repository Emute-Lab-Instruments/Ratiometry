
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

#define _RATIO_PLAY_MODE_ 0
#define _CV_PLAY_MODE_ 1
#define _NO_MORE_MODES_ 2

int currentPlayMode = _RATIO_PLAY_MODE_;

bool core1_separate_stack = true;

//objects for the Encoder Event Models
EncoderEventModel* currentEventModel;      //pointer for the currently active model for encoder input

EncoderEventModel ratioEvents[6] =  {EncoderEventModel(),EncoderEventModel(),EncoderEventModel(),EncoderEventModel(),EncoderEventModel(),EncoderEventModel()}; //6 Ratio event workspaces when declared!
//EncoderEventModel cvEvents = (EncoderEventModel) OtherEventModel();    //6 CV output event workspaces
//EncoderEventModel cvEvents = (EncoderEventModel) MutesEventModel();    //Mutes event workspaces ** not coed **
EncoderEventModel cvEvents[6] =  {EncoderEventModel(),EncoderEventModel(),EncoderEventModel(),EncoderEventModel(),EncoderEventModel(),EncoderEventModel()}; //6 Ratio event workspaces when declared!

//decalre worksoacecounters
int currentRatioModel = 0;
int currentCVModel = 0;

//neo pixels declarations
#define PIN        9 // 
#define NUMPIXELS  6 // 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t pixelBandColours[6] = {pixels.Color(0,0,255), pixels.Color(255,255,0), pixels.Color(0,255,0), pixels.Color(255,255,255),pixels.Color(230,80,0), pixels.Color(255,9,0) }; 


DisplayPortal display;
//DisplayPortal ratioDisplay;
//CVDisplayPortal cvDisplay;

namespace controls {
 // static int encoderValues[6] = {4,5,4,3,6,4};;
 // static bool encoderSwitches[6] = {0,0,0,0,0,0};
 // static float pulse_width = 0.5;
  //static float phaseRate = 0.001;
};

namespace outputs {
  static bool triggers[6] = {0,0,0,0,0,0};
  static int currentBeat[6] = {0,0,0,0,0,0};
  static int currentCVNote[6] = {0,0,0,0,0,0};
  static double currentCV[8] = {0,0,0,0,0,0,0,0};
};

namespace switches {
  static double t_trigger[9] = {0,0,0,0,0,0,0,0,0};
  static bool b_state[9] = {1,1,1,1,1,1,1,1,1};
  static int event_state[9] = {0,0,0,0,0,0,0,0,0};
};

bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  // tft.fillScreen(TFT_BLACK);
  // tft.setCursor(120,120);
  // tft.setTextColor(TFT_WHITE);
  // tft.println(controls::encoderValues[0]);

  // tft.setCursor(120,150);
  // tft.println(controlValues[0]);
  //display.update(controls::encoderValues, controls::pulse_width);

  //if (currentPlayMode == _RATIO_PLAY_MODE_) 
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
  currentEventModel->changeEncoder1(change);
  display.setReDraw();
}
//red
void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  currentEventModel->changeEncoder6(change);
  display.setReDraw();
}

//orange
void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
   currentEventModel->changeEncoder5(change);
  display.setReDraw();
}
//white
void encoder4_callback() {
  int change = read_rotary(enc4Code, enc4Store, ENCODER4_A_PIN, ENCODER4_B_PIN);
  currentEventModel->changeEncoder4(change);
  display.setReDraw();
}

//green
void encoder5_callback() {
  int change = read_rotary(enc5Code, enc5Store, ENCODER5_A_PIN, ENCODER5_B_PIN);
  currentEventModel->changeEncoder3(change);
  display.setReDraw();
}

//yellowish
void encoder6_callback() {
  int change = read_rotary(enc6Code, enc6Store, ENCODER6_A_PIN, ENCODER6_B_PIN);
  currentEventModel->changeEncoder2(change);
  display.setReDraw();
}

//left - pulse width
void encoder7_callback() {
  int change = read_rotary(enc7Code, enc7Store, ENCODER7_A_PIN, ENCODER7_B_PIN);
  //currentEventModel->changeEncoder7(change);
  ratioEvents[currentRatioModel].changeEncoder7(change);
  display.setReDraw();
}

//Right - phaseRate
void encoder8_callback() {
  int change = read_rotary(enc8Code, enc8Store, ENCODER8_A_PIN, ENCODER8_B_PIN);
  //currentEventModel->changeEncoder8( change);
  ratioEvents[currentRatioModel].changeEncoder8( change);
//  display.setReDraw(); No dosplay up date for speed changes ** might need to check on Model Mode later in dev if the ENC gets used**
}

void processEncoderSwitch(int n, int ENCODER_SWITCH) {
  bool bNow = digitalRead(ENCODER_SWITCH); 
  float t_since = millis() - switches::t_trigger[n];
  
  if ( t_since < 30) return;

  //Serial.println(bNow);
  if ((bNow == 0) && (switches::event_state[n] == 0)) {
    switches::t_trigger[n] = millis(); 
    switches::event_state[n] = 1;
  }
  if ((switches::b_state[n] == 0) && (bNow == 1)&& (switches::event_state[n] == 1)) {
    if (t_since>300) switches::event_state[n] = 3;
    else switches::event_state[n] = 2;
  }
  switches::b_state[n] = bNow;
}

void encoder1_switch_callback() {processEncoderSwitch(0,ENCODER1_SWITCH);}
void encoder2_switch_callback() {processEncoderSwitch(1,ENCODER2_SWITCH);}
void encoder3_switch_callback() {processEncoderSwitch(2,ENCODER3_SWITCH);}
void encoder4_switch_callback() {processEncoderSwitch(3,ENCODER4_SWITCH);}
void encoder5_switch_callback() {processEncoderSwitch(4,ENCODER5_SWITCH);}
void encoder6_switch_callback() {processEncoderSwitch(5,ENCODER6_SWITCH);}

void modeSwitch_switch_callback(){processEncoderSwitch(8,MODESWITCH);}

//void encoder6_switch_callback() {processEncoderSwitch(5,ENCODER6_SWITCH);}

// void encoder1_switch_callback() {
//   bool bNow = digitalRead(ENCODER1_SWITCH); 
//   float t_since = millis() - switches::t_trigger[0];
  
//   if ( t_since < 30) return;

//   //Serial.println(bNow);
//   if ((bNow == 0) && (switches::event_state[0] == 0)) {
//     switches::t_trigger[0] = millis(); 
//     switches::event_state[0] = 1;
//   }
//   if ((switches::b_state[0] == 0) && (bNow == 1)&& (switches::event_state[0] == 1)) {
//     if (t_since>300) switches::event_state[0] = 3;
//     else switches::event_state[0] = 2;
//   }
//   switches::b_state[0] = bNow;
// }

// void encoder2_switch_callback() {
//   bool bNow = digitalRead(ENCODER2_SWITCH); 
//   float t_since = millis() - switches::t_trigger[1];
  
//   if ( t_since < 30) return;

//   //Serial.println(bNow);
//   if ((bNow == 0) && (switches::event_state[1] == 0)) {
//     switches::t_trigger[1] = millis(); 
//     switches::event_state[1] = 1;
//   }
//   if ((switches::b_state[1] == 0) && (bNow == 1)&& (switches::event_state[1] == 1)) {
//     if (t_since>300) switches::event_state[1] = 3;
//     else switches::event_state[1] = 2;
//   }
//   switches::b_state[1] = bNow;
// }


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
  //first process encoders
  for(int i = 0; i < 7; i++){
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
        if (currentPlayMode == _RATIO_PLAY_MODE_) setCurrentRatioEventModel(i);
        else if (currentPlayMode == _CV_PLAY_MODE_) setCurrentCVEventModel(i);
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

  // process the mode button event
  int i = 8;
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
      if (currentPlayMode == _RATIO_PLAY_MODE_)   setCurrentCVEventModel(currentCVModel);
      else if (currentPlayMode == _CV_PLAY_MODE_) setCurrentRatioEventModel(currentRatioModel);
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
  return true;
}

void stepCVNote(int ch){
  outputs::currentCVNote[ch] ++;
  if (outputs::currentCVNote[ch]>5) outputs::currentCVNote[ch] = 0;
  outputs::currentCV[ch] = cvEvents[currentCVModel].encoderValues[outputs::currentCVNote[ch]];
}

bool __not_in_flash_func(dspLoop)(__unused struct repeating_timer *t) {
  ratiodata::phase += *ratiodata::phaseRate; //0.002;
  if (ratiodata::phase > 1.0) {
    ratiodata::phase -= 1.0;
  }
  if (ratiodata::phase <0) {
    ratiodata::phase += 1.0;
  }

  bool currentGateOut = 0;
  int pwmResolution = 65535;
  digitalWrite(PULSEOUT0_PIN, ratiodata::phase < *ratiodata::pulseWidth);
  //outputs::triggers[5] = ratiodata::phase < *ratiodata::pulseWidth; 
  currentGateOut = ratiodata::phase < *ratiodata::pulseWidth;
  //if (outputs::triggers[5] != currentGateOut) stepCVNote(5); // step CV note on trasition of output 0-1 and 1-0 i.e. includes cpulsewidth
  if ((outputs::triggers[5]==0) && (currentGateOut == 1)) stepCVNote(5); // step CV at end of ratio just 0-1
  analogWrite(CVOUT0_PIN, outputs::currentCV[5] * pwmResolution);
  outputs::triggers[5] = currentGateOut;
  outputs::currentBeat[5] = 0;
  
  auto ratios1 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1]});
  int seq1 = ratioseq::rpulse(ratios1, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT1_PIN, seq1/10);
  currentGateOut = seq1/10;
  //if (outputs::triggers[4] != currentGateOut) stepCVNote(4); // step CV note on trasition of output 0-1 and 1-0 i.e. includes cpulsewidth
  if (outputs::currentBeat[4] != (seq1 % 10)) stepCVNote(4); // step CV at end of ratio
  analogWrite(CVOUT1_PIN, outputs::currentCV[4] * pwmResolution);
  outputs::triggers[4] = currentGateOut;
  outputs::currentBeat[4] = seq1 % 10;

  auto ratios2 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2]});
  int seq2 = ratioseq::rpulse(ratios2, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT2_PIN, seq2/10);
  currentGateOut = seq2/10;
  //if (outputs::triggers[3] != currentGateOut) stepCVNote(3);// step CV note on trasition of output 0-1 and 1-0 i.e. includes cpulsewidth
  if (outputs::currentBeat[3] != (seq2 % 10)) stepCVNote(3); // step CV at end of ratio
  analogWrite(CVOUT2_PIN, outputs::currentCV[3] * pwmResolution);
  outputs::triggers[3] = currentGateOut;
  outputs::currentBeat[3] = seq2 % 10;

  auto ratios3 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3]});
  int seq3 = ratioseq::rpulse(ratios3, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT3_PIN, seq3/10);
  //outputs::triggers[2] = seq3/10;
  currentGateOut = seq3/10;
  //if (outputs::triggers[2] != currentGateOut) stepCVNote(2);// step CV note on trasition of output 0-1 and 1-0 i.e. includes cpulsewidth
  if (outputs::currentBeat[2] != (seq3 % 10)) stepCVNote(2); // step CV at end of ratio
  analogWrite(CVOUT3_PIN, outputs::currentCV[2] * pwmResolution);
  outputs::triggers[2] = currentGateOut;
  outputs::currentBeat[2] = seq3 % 10; 

  auto ratios4 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3],ratiodata::ratios[4]});
  int seq4 = ratioseq::rpulse(ratios4, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT4_PIN, seq4/10);
  //outputs::triggers[1] = seq4/10;
  currentGateOut = seq4/10;
  //if (outputs::triggers[1] != currentGateOut) stepCVNote(1);// step CV note on trasition of output 0-1 and 1-0 i.e. includes cpulsewidth
  if (outputs::currentBeat[1] != (seq4 % 10)) stepCVNote(1); // step CV at end of ratio
  analogWrite(CVOUT4_PIN, outputs::currentCV[1] * pwmResolution);
  outputs::triggers[1] = currentGateOut;
  outputs::currentBeat[1] = seq4 % 10; 

  auto ratios5 = std::vector<double>({ratiodata::ratios[0],ratiodata::ratios[1],ratiodata::ratios[2],ratiodata::ratios[3],ratiodata::ratios[4],ratiodata::ratios[5]});
  int seq5 = ratioseq::rpulse(ratios5, *ratiodata::pulseWidth, ratiodata::phase);
  digitalWrite(PULSEOUT5_PIN, seq5/10);
  //outputs::triggers[0] = seq5/10;
  currentGateOut = seq5/10;
  //if (outputs::triggers[0] != currentGateOut) stepCVNote(0);// step CV note on trasition of output 0-1 and 1-0 i.e. includes cpulsewidth
  if (outputs::currentBeat[0] != (seq5 % 10)) stepCVNote(0); // step CV at end of ratio
  analogWrite(CVOUT5_PIN, outputs::currentCV[0] * pwmResolution);
  outputs::triggers[0] = currentGateOut;
  outputs::currentBeat[0] = seq5 % 10; 

  return true;
}

bool __not_in_flash_func (updateNeoPixels)(__unused struct repeating_timer *t){
 for(int i = 0; i < NUMPIXELS; i++){
  switch (currentPlayMode){
    case _RATIO_PLAY_MODE_:
      pixels.setPixelColor(i, pixelBandColours[outputs::currentBeat[i]]* outputs::triggers[i]);
      break;
    case _CV_PLAY_MODE_:
      pixels.setPixelColor(i, pixelBandColours[outputs::currentCVNote[i]]);
      break;
  }
    //cvEvents[currentCVModel].encoderValues[outputs::currentBeat[i]] *
 }
 pixels.show();
 return true;
}

//this loads default values - later load from storage
void loadRatioModels(){
  
  double maximums[8] = {32,32,32,32,32,32, 0.95, 0.004};
  double minimums[8] = {1,1,1,1,1,1, 0.05, -0.004};
  double steps[8] = {1,1,1,1,1,1, 0.05, 0.00005};

  double values[8] = {4,5,4,3,6,4, 0.9, 0.002};
  ratioEvents[0].setEncoderValues(values);

  double values1[8] = {10,5,7,8,1,1, 0.5, 0.002};
  ratioEvents[1].setEncoderValues(values1);

  double values2[8] = {10,25,7,8,5,21, 0.3, 0.002};
  ratioEvents[2].setEncoderValues(values2);

  double values3[8] = {1,25,7,8,10,1, 0.6, 0.002};
  ratioEvents[3].setEncoderValues(values3);

  double values4[8] = {15,15,7,8,15,15, 0.3, 0.002};
  ratioEvents[4].setEncoderValues(values4);

  double values5[8] = {10,5,7,8,1,32, 0.8, 0.002};
  ratioEvents[5].setEncoderValues(values5);

  for (int i = 0; i<6; i++){
    ratioEvents[i].setEncoderMaximums(maximums);
    ratioEvents[i].setEncoderMinimums(minimums);
    ratioEvents[i].setEncoderSteps(steps);
  }
}

//load some models into the CV workspaces
void loadCVModels(){
  double values[8] = {0,0,0,0,0,0,0,0};
  for (int ch = 0; ch<6; ch++){
    for (int s = 0; s<6; s++){
      cvEvents[ch].encoderValues[s]= random(0,100)/100.0;
    }
    cvEvents[ch].encoderValues[6] = 0.5;
    cvEvents[ch].encoderValues[7] = 0.5;

  }  
}

void setCurrentRatioEventModel(int n){
  currentEventModel = &ratioEvents[n];
  ratiodata::ratios = currentEventModel->encoderValues;

  //reset timebeats and CV note sequencing ** not the best but we'll do this for now
  ratiodata::phase=0;
  for (int i =0;i<6;i++) {outputs::currentBeat[i]=0; outputs::currentCVNote[i]=0;}

  ratiodata::pulseWidth=&currentEventModel->encoderValues[6];
  ratiodata::sampleRate = 1000.0;
  ratiodata::phaseRate = &currentEventModel->encoderValues[7];
  currentRatioModel = n;
  display.setReDraw();
  currentPlayMode = _RATIO_PLAY_MODE_;
}

void setCurrentCVEventModel(int n){
  currentEventModel = &cvEvents[n];
  currentCVModel = n;
  //reset time, beats and CV note sequencing ** not the best but we'll do this for now
  ratiodata::phase=0;
  for (int i =0;i<6;i++) {outputs::currentBeat[i]=0; outputs::currentCVNote[i]=0;}

  display.setReDraw();
  currentPlayMode = _CV_PLAY_MODE_;
}

void setup() {

  analogWriteResolution(16);
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

  //load CV workspaces
  loadCVModels();

  //point current model to first workspace
  setCurrentRatioEventModel(0);

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

  pinMode(MODESWITCH, INPUT_PULLUP);

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
  attachInterrupt(digitalPinToInterrupt(ENCODER2_SWITCH), encoder2_switch_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_SWITCH), encoder3_switch_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_SWITCH), encoder4_switch_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER5_SWITCH), encoder5_switch_callback,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER6_SWITCH), encoder6_switch_callback, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MODESWITCH), modeSwitch_switch_callback, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(ENCODER3_SWITCH), encoder3_switch_callback,
  //                   CHANGE);

  pinMode(PULSEOUT0_PIN, OUTPUT);
  pinMode(PULSEOUT1_PIN, OUTPUT);
  pinMode(PULSEOUT2_PIN, OUTPUT);
  pinMode(PULSEOUT3_PIN, OUTPUT);
  pinMode(PULSEOUT4_PIN, OUTPUT);
  pinMode(PULSEOUT5_PIN, OUTPUT);

  pinMode(CVOUT0_PIN, OUTPUT);
  pinMode(CVOUT1_PIN, OUTPUT);
  pinMode(CVOUT2_PIN, OUTPUT);
  pinMode(CVOUT3_PIN, OUTPUT);
  pinMode(CVOUT4_PIN, OUTPUT);
  pinMode(CVOUT5_PIN, OUTPUT);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.setBrightness(10);

  add_repeating_timer_ms(30, displayUpdate, NULL, &timerDisplayProcessor);
  add_repeating_timer_ms(30, updateNeoPixels, NULL, &timerPixelDisplayProcessor);
  add_repeating_timer_ms(20, processSwitchEvents, NULL, &timerSwitchEventsProcessor);


  //display intial ratios
  //display.update(controls::encoderValues, controls::pulse_width);

  add_repeating_timer_ms(-2, dspLoop, NULL, &timerDSPLoop);
  //display = &ratioDisplay;
  //display =  &cvDisplay;

  //currentPlayMode = _RATIO_PLAY_MODE_;
  //setCurrentCVEventModel(0);

  display.setReDraw();
}


int count=0;
void loop() {
  //__wfi();
  //display.update(controls::encoderValues, controls::pulse_width);
  //display.update(ratiodata::ratios, *ratiodata::pulseWidth);

  switch(currentPlayMode){
    case _RATIO_PLAY_MODE_:
      display.update(ratiodata::ratios, *ratiodata::pulseWidth);

      break;

    case _CV_PLAY_MODE_:
      //display.update(outputs::currentCV, *ratiodata::pulseWidth);
      //cvDisplay.setReDraw();
      display.cvupdate(currentEventModel->encoderValues, *ratiodata::pulseWidth);

      break;  
    default:
      break;
  }

  delay(20);
}


void setup1() {
}

void loop1() {
  __wfi();
}
