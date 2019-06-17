#include "tools.h"
#include <iostream>
#include <string>

using Eigen::VectorXd;
using std::vector;
using std::string;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  //check the validity of the inputs
  try {
    if (estimations.size() == 0){
      throw string("The estimation vector size should not be zero!!");
    }
    else if (estimations.size() != ground_truth.size()){
      throw string("The estimation vector size should equal ground truth vector size!!");
    }
  }
  catch (string &e){
    std::cout << e << std::endl;
    return rmse;
  }

  // compute RMSE
  if (estimations.size() == 1){
    total_residual = VectorXd(4);
    total_residual << 0, 0, 0, 0;
  }

  int last_index = estimations.size() - 1;
  VectorXd residual = estimations[last_index] - ground_truth[last_index];
  residual = residual.array() * residual.array();
  total_residual += residual;
  rmse = total_residual / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
