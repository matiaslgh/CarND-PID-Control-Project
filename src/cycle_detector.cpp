#include <math.h>
#include "cycle_detector.h"
#include <iostream>

CycleDetector::CycleDetector() {
  value = 0;
  count = 0;
  total_error = 0;
}

bool CycleDetector::isCrossingZero(double new_value) {
  return  (value < 0 && new_value >= 0) ||
          (value > 0 && new_value <= 0) ||
          (value == 0 && new_value != 0);
}

bool CycleDetector::isHalfCycleCompleted(double new_value) {
  if (isCrossingZero(new_value) && count != 0) {
    value = new_value;
    return true;
  }
  value = new_value;
  return false;
}

void CycleDetector::cleanState() {
  count = 0;
  total_error = 0;
}

void CycleDetector::update(double error) {
  count++;
  total_error += fabs(error);
}

double CycleDetector::getAverageError() {
  return total_error / count;
}