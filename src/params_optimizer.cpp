#include "params_optimizer.h"
#include <vector>
#include <string>
#include <iostream>

ParamsOptimizer::ParamsOptimizer(std::vector<double> initial_params, double min_tolerance) {
  params_ = initial_params;
  min_tolerance_ = min_tolerance;
  was_last_param_decreased_ = false;
  param_index_to_update_ = 0;
  best_error_ = 999999;
  for (int i = 0; i < initial_params.size(); i++) {
    param_deltas_.push_back(1);
  }
}

double ParamsOptimizer::getTolerance() {
  double tolerance = 0;
  for (int i=0; i < param_deltas_.size(); i++) {
    tolerance += param_deltas_[i];
  }
  return tolerance;
}

void ParamsOptimizer::setNextParamIndexToUpdate() {
  param_index_to_update_ = param_index_to_update_ == params_.size() - 1 ? 0 : param_index_to_update_ + 1;
}

void ParamsOptimizer::handleBetterError(double error) {
  best_error_ = error;
  param_deltas_[param_index_to_update_] *= 1.1;
  setNextParamIndexToUpdate();
  params_[param_index_to_update_] += param_deltas_[param_index_to_update_];
  was_last_param_decreased_ = false;
}

void logIfSameParamIndexes(int target_param_index, int param_index, std::string msg) {
  if (target_param_index == param_index) {
    std::cout << msg << std::endl;
  }
}

std::vector<double> ParamsOptimizer::getParams(double error) {
  if (getTolerance() <= min_tolerance_) {
    return params_;
  }

  if (was_last_param_decreased_) {
    logIfSameParamIndexes(0, param_index_to_update_, "was_last_param_decreased_=true");
    if (error < best_error_) {
      logIfSameParamIndexes(0, param_index_to_update_, "error < best_error_=true");
      handleBetterError(error);
    } else {
      logIfSameParamIndexes(0, param_index_to_update_, "error < best_error_=false");
      params_[param_index_to_update_] += param_deltas_[param_index_to_update_];
      param_deltas_[param_index_to_update_] *= 0.9;
      setNextParamIndexToUpdate();
      params_[param_index_to_update_] += param_deltas_[param_index_to_update_];
      was_last_param_decreased_ = false;
    }
  } else {
    logIfSameParamIndexes(0, param_index_to_update_, "was_last_param_decreased_=false");
    if (error < best_error_) {
      logIfSameParamIndexes(0, param_index_to_update_, "error < best_error_=true");
      handleBetterError(error);
    } else {
      logIfSameParamIndexes(0, param_index_to_update_, "error < best_error_=false");
      params_[param_index_to_update_] -= 2 * param_deltas_[param_index_to_update_];
      was_last_param_decreased_ = true;
    }
  }
  logIfSameParamIndexes(0, param_index_to_update_, "new_param: " + std::to_string(params_[0]));

  return params_;
}
