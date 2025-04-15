// ratios code taken from uSEQ

#include <vector>


namespace ratiodata {
  double* ratios;
  double phase;
  double* pulseWidth;
  double sampleRate;
  double* phaseRate;
}

class ratioseq {
public:

  static constexpr double RATIOMAX = 32;
  static constexpr double RATIOMIN = 2;

  static int rpulse(std::vector<double> &ratios, float pulseWidth, float phase) {
    bool trig=0;
    double ratioSum = 0;
    for(double v: ratios) {
        ratioSum += v;
    }
    double phaseAdj = ratioSum * phase;
    double accumulatedSum=0;
    double lastAccumulatedSum=0;
    int beat = 0;
    for(double v : ratios) {
        accumulatedSum += v;
        if (phaseAdj <= accumulatedSum) {
            //check pulse width
            double beatPhase = (phaseAdj - lastAccumulatedSum) / (accumulatedSum - lastAccumulatedSum);
            trig = beatPhase <= pulseWidth;
            break;
        }
        lastAccumulatedSum = accumulatedSum;
        beat++;
    }
    return (10*trig) + beat;    
  }
};