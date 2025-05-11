//DisplayPortals.h
//Steve Symons, Emute Lab Instruments 2025
//University of Sussex

//series of classes for different display modes


#include <TFT_eSPI.h>
#define ELI_BLUE 0x0007
#define ELI_PINK 0xFC9F

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
#define SCREEN_WIDTH tft.width()    //
#define SCREEN_HEIGHT tft.height()  // Taille de l'écran

// Create two sprites for a DMA toggle buffer
TFT_eSprite spr[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};

// Pointers to start of Sprites in RAM (these are then "image" pointers)
uint16_t* sprPtr[2];


//core class but also used for Ratios atm
class DisplayPortal {
public:
  uint16_t fg_color = TFT_RED;
  uint16_t bg_color = TFT_BLACK;       // This is the background colour used for smoothing (anti-aliasing)
  const int bandColour[6] = {TFT_BLUE,TFT_YELLOW , TFT_GREEN , TFT_WHITE,TFT_ORANGE ,TFT_RED };

  uint8_t minRadius  = 40; // Outer arc radius
  uint8_t bandThickness  = 10;     // Thickness
  uint8_t bandSeparation = 5;

  bool bReDraw = false;
  float lastTimerPhase = 0;

 void drawTimer(float phase){
  //for(int outputBand = 1; outputBand < 7; outputBand++){
    tft.drawArc(120, 120,28, 30-bandSeparation, 360*lastTimerPhase-4, 360*lastTimerPhase , bg_color, bg_color);
    tft.drawArc(120, 120,28, 30-bandSeparation, 360*phase-4, 360*phase , ELI_PINK, ELI_PINK);
  //}
  lastTimerPhase = phase;
 }
 
 void drawSegment(bool sel, uint8_t radius, uint8_t thickness, int startAngle, int stopAngle, int colour, float pulse_width ){
  // clear old band

  //delay(10);
  // two drawing styles this generates too many lines
  //tft.drawSmoothArc(120, 120, radius, radius - thickness, startAngle, stopAngle, colour, bg_color);
  //tft.drawSmoothArc(120, 120, radius-2, radius - thickness, startAngle + (stopAngle-startAngle)*(1-pulse_width), stopAngle, bg_color, bg_color);

  //cleaner
  spr[sel].drawArc(120, 120, radius - thickness/2, radius - thickness, startAngle, stopAngle, colour, bg_color);
  spr[sel].drawArc(120, 120, radius, radius - thickness, startAngle, startAngle + (stopAngle-startAngle)*pulse_width, colour, bg_color);
  
 }
 void update(double ratios[], float pulse_width){
  //do nothing if no changes to show
  if (!bReDraw) return;
  drawUpdate(0,ratios,pulse_width);
  drawUpdate(1,ratios,pulse_width);
  
  bReDraw = false;
 }

void drawUpdate(bool sel,double ratios[], float pulse_width){
  //this is sooo slow
  spr[sel].fillScreen(TFT_BLACK);

  int totalBandMetry = 0;
  int offsetAngle = 0;
  float thisRatioAngle, endPoint;
  for(int outputBand = 1; outputBand < 7; outputBand++){
    
    totalBandMetry = 0;
    offsetAngle = 0;
    for(int i = 0; i < outputBand; i++) totalBandMetry += ratios[i];
    //spr[sel].drawArc(120, 120, minRadius+((bandSeparation+bandThickness)*(outputBand-1)), minRadius+((bandSeparation+bandThickness)*(outputBand-1)) - bandThickness, 0, 360, bg_color, bg_color);

    for(int thisRatio = 0; thisRatio < outputBand; thisRatio++){
      thisRatioAngle = ( 360 * (ratios[thisRatio]/float(totalBandMetry)));
      endPoint = thisRatioAngle + offsetAngle;
      if (thisRatio == (outputBand-1)) endPoint = 360;
      drawSegment(sel, minRadius+((bandSeparation+bandThickness)*(outputBand-1)) , bandThickness, offsetAngle, endPoint, bandColour[thisRatio], pulse_width);
      offsetAngle += thisRatioAngle; 
    }

  }
  // draw current work space and ratio workspace selection gtaphics here

  
  drawTimer(ratiodata::phase); //this need not be here as uses TFT so not drawn twice - draw once at end in Update?
  tft.pushImageDMA(0, sel * SCREEN_HEIGHT / 2, SCREEN_WIDTH, SCREEN_HEIGHT / 2, sprPtr[sel]);
  
  }

  void setReDraw(){bReDraw = true;}

};