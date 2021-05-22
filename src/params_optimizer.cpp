#include "params_optimizer.h"
#include <vector>
#include <string>
#include <iostream>

ParamsOptimizer::ParamsOptimizer(std::vector<double> initial_params, double min_tolerance) {
  params_ = initial_params;
  min_tolerance_ = min_tolerance;
  was_last_param_decreased_ = false;
  param_index_to_update_ = 0;
  best_error_ = 0.82;
  // for (int i = 0; i < initial_params.size(); i++) {
  //   param_deltas_.push_back(1);
  // }

  // Temporary using these alphas to fine tune and avoid big changes
  param_deltas_.push_back(0.001);
  param_deltas_.push_back(0.00005);
  param_deltas_.push_back(0.001);
}

double ParamsOptimizer::getTolerance() {
  double tolerance = 0;
  for (int i=0; i < param_deltas_.size(); i++) {
    tolerance += param_deltas_[i];
  }
  return tolerance;
}

void ParamsOptimizer::setNextParamIndexToUpdate() {
  log("Updating index_to_update from " + std::to_string(param_index_to_update_) + " to " + std::to_string(param_index_to_update_ == params_.size() - 1 ? 0 : param_index_to_update_ + 1));
  param_index_to_update_ = param_index_to_update_ == params_.size() - 1 ? 0 : param_index_to_update_ + 1;
}

void ParamsOptimizer::log(std::string msg) {
  if (param_index_to_update_ == 0) {
    std::cout << "Kp - " << msg << std::endl;
  } else if (param_index_to_update_ == 1) {
    std::cout << "Ki - " << msg << std::endl;
  } else if (param_index_to_update_ == 2) {
    std::cout << "Kd - " << msg << std::endl;
  } else {
    std::cout << "THIS SHOULD NEVER BE SHOWN - " << msg << std::endl;
  }
}

void ParamsOptimizer::handleBetterError(double error) {
  log("Update best_error");
  best_error_ = error;
  setNextParamIndexToUpdate();
  log("Increasing param from " + std::to_string(params_[param_index_to_update_]) + " to " + std::to_string(params_[param_index_to_update_] + param_deltas_[param_index_to_update_]));
  params_[param_index_to_update_] += param_deltas_[param_index_to_update_];
  log("Set was_last_param_decreased_ = false");
  was_last_param_decreased_ = false;
}

std::vector<double> ParamsOptimizer::getParams(double error) {
  log("(1) error: " + std::to_string(error));
  if (getTolerance() <= min_tolerance_) {
    std::cout << "(2) Min tolerance reached - Best params = " << params_[0] << " | " << params_[1] << " | " << params_[2] << " | " << std::endl;
    return params_;
  }

  if (was_last_param_decreased_) {
    log("(2) last_param_decreased: true");
    if (error < best_error_) {
      log("(3) NEW BEST ERROR - error: " + std::to_string(error) + " | best_error_: " + std::to_string(best_error_));
      log("(4) Updating delta from " + std::to_string(param_deltas_[param_index_to_update_]) + " to " + std::to_string(param_deltas_[param_index_to_update_] * 1.05));
      param_deltas_[param_index_to_update_] *= 1.05;
      handleBetterError(error);
    } else {
      log("(5) WORSE - error: " + std::to_string(error) + " | best_error_: " + std::to_string(best_error_));
      log("(6) Fix back param from " + std::to_string(params_[param_index_to_update_]) + " to " + std::to_string(params_[param_index_to_update_] + param_deltas_[param_index_to_update_]));
      params_[param_index_to_update_] += param_deltas_[param_index_to_update_];
      log("(7) Updating delta from " + std::to_string(param_deltas_[param_index_to_update_]) + " to " + std::to_string(param_deltas_[param_index_to_update_] * 0.95));
      param_deltas_[param_index_to_update_] *= 0.95;
      setNextParamIndexToUpdate();
      log("(8) Increasing param from " + std::to_string(params_[param_index_to_update_]) + " to " + std::to_string(params_[param_index_to_update_] + param_deltas_[param_index_to_update_]));
      params_[param_index_to_update_] += param_deltas_[param_index_to_update_];
      log("(9) Set was_last_param_decreased_ = false");
      was_last_param_decreased_ = false;
    }
  } else {
    log("(10) last_param_decreased: false");
    if (error < best_error_) {
      log("(11) NEW BEST ERROR - error: " + std::to_string(error) + " | best_error_: " + std::to_string(best_error_));
      log("(12) Updating delta from " + std::to_string(param_deltas_[param_index_to_update_]) + " to " + std::to_string(param_deltas_[param_index_to_update_] * 1.1));
      param_deltas_[param_index_to_update_] *= 1.1;
      handleBetterError(error);
    } else {
      log("(13) WORSE - error: " + std::to_string(error) + " | best_error_: " + std::to_string(best_error_));
      log("(14) Decreasing param from " + std::to_string(params_[param_index_to_update_]) + " to " + std::to_string(params_[param_index_to_update_] - 2 * param_deltas_[param_index_to_update_]));
      params_[param_index_to_update_] -= 2 * param_deltas_[param_index_to_update_];
      log("(15) Set was_last_param_decreased_ = true");
      was_last_param_decreased_ = true;
    }
  }
  log("(16) deltas: " + std::to_string(param_deltas_[0]) + " | " + std::to_string(param_deltas_[1]) + " | " + std::to_string(param_deltas_[2]));
  log("(17) Return: " + std::to_string(params_[0]) + " | " + std::to_string(params_[1]) + " | " + std::to_string(params_[2]));
  std::cout << "-----------------------------------------------" << std::endl;

  return params_;
}
