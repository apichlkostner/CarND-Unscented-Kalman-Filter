#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd mse = VectorXd::Zero(4);

  assert(estimations.size() == ground_truth.size());

  for (size_t i = 0; i < estimations.size(); i++) {
    VectorXd delta = estimations[i] - ground_truth[i];
    mse += delta.cwiseProduct(delta);
  }

  mse /= estimations.size();

  return mse.array().sqrt();
}