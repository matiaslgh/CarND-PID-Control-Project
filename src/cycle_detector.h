#ifndef CYCLE_DETECTOR_H
#define CYCLE_DETECTOR_H

#include <vector>

class CycleDetector {
 private:
  double value;
  double count;
  double total_error;
  bool isCrossingZero(double new_value);

 public:
  CycleDetector();
  bool isHalfCycleCompleted(double new_value);
  void cleanState();
  void update(double error);
  double getAverageError();
};

#endif /* CYCLE_DETECTOR_H */