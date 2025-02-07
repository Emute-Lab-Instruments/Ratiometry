#include <vector>


namespace ratiodata {
  std::vector<double> ratios;
  double phase;
  double pulseWidth;
  double sampleRate;
}

class ratioseq {
public:

  static constexpr double RATIOMAX = 16;
  static constexpr double RATIOMIN = 1;

  static bool rpulse(std::vector<double> &ratios, float pulseWidth, float phase) {
    bool trig=0;
    double ratioSum = 0;
    for(double v: ratios) {
        ratioSum += v;
    }
    double phaseAdj = ratioSum * phase;
    double accumulatedSum=0;
    double lastAccumulatedSum=0;
    for(double v : ratios) {
        accumulatedSum += v;
        if (phaseAdj <= accumulatedSum) {
            //check pulse width
            double beatPhase = (phaseAdj - lastAccumulatedSum) / (accumulatedSum - lastAccumulatedSum);
            trig = beatPhase <= pulseWidth;
            break;
        }
        lastAccumulatedSum = accumulatedSum;
    }
    return trig;    
  }
};