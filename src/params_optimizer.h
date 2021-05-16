#ifndef PARAMS_OPTIMIZER_H
#define PARAMS_OPTIMIZER_H

#include <vector>

class ParamsOptimizer {
 private:
  double best_error_;
  int param_index_to_update_;
  double min_tolerance_;
  bool was_last_param_decreased_;
  std::vector<double> params_;
  std::vector<double> param_deltas_;
  double getTolerance();
  void handleBetterError(double error);
  void setNextParamIndexToUpdate();

 public:
  ParamsOptimizer(std::vector<double> initial_params, double min_tolerance);
  std::vector<double> getParams(double error);
};

#endif /* PARAMS_OPTIMIZER_H */