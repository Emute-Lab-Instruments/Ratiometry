// EncoderEventModels
//Steve Symons, Emute Lab Instruments 2025
//University of Sussex

// Class to handle the encode events - holding max mon ans step values
// core class model
class EncoderEventModel {
  public:
  double encoderValues[8]={0,0,0,0,0,0,0,0};
  double maxValues[8] = {1,1,1,1,1,1,1,1};
  double minValues[8] = {0,0,0,0,0,0,0,0};
  double stepValues[8] = {0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05};

  void setEncoderValues(double* val){for(int i=0;i<8;i++) encoderValues[i] = val[i];}
  void setEncoderMaximums(double* val){for(int i=0;i<8;i++) maxValues[i] = val[i];}
  void setEncoderMinimums(double* val){for(int i=0;i<8;i++) minValues[i] = val[i];}
  void setEncoderSteps(double* val){for(int i=0;i<8;i++) stepValues[i] = val[i];}

  double maxMin(double val, double max, double min){
    val = std::max(val, min);
    val = std::min(val, max);
    return val;
  }

  void changeEncoder1(int change){encoderValues[0] += (stepValues[0]*change); encoderValues[0] = maxMin(encoderValues[0], maxValues[0], minValues[0]);}
  void changeEncoder2(int change){encoderValues[1] += (stepValues[1]*change); encoderValues[1] = maxMin(encoderValues[1], maxValues[1], minValues[1]);}
  void changeEncoder3(int change){encoderValues[2] += (stepValues[2]*change); encoderValues[2] = maxMin(encoderValues[2], maxValues[2], minValues[2]);}
  void changeEncoder4(int change){encoderValues[3] += (stepValues[3]*change); encoderValues[3] = maxMin(encoderValues[3], maxValues[3], minValues[3]);}
  void changeEncoder5(int change){encoderValues[4] += (stepValues[4]*change); encoderValues[4] = maxMin(encoderValues[4], maxValues[4], minValues[4]);}
  void changeEncoder6(int change){encoderValues[5] += (stepValues[5]*change); encoderValues[5] = maxMin(encoderValues[5], maxValues[5], minValues[5]);}
  void changeEncoder7(int change){encoderValues[6] += (stepValues[6]*change); encoderValues[6] = maxMin(encoderValues[6], maxValues[6], minValues[6]);}
  void changeEncoder8(int change){encoderValues[7] += (stepValues[7]*change); encoderValues[7] = maxMin(encoderValues[7], maxValues[7], minValues[7]);}


};
