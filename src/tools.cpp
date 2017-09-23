#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**

    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0., 0., 0., 0.;

  if(estimations.size() != ground_truth.size()) {
    cout << "error: estimations != ground_truth dimentions" << endl;
    return rmse;
  }
  if( estimations.size() == 0 ) {
    cout << "error: estimations size == 0" << endl;
    return rmse;
  }
  
  for(auto i=0; i< estimations.size(); ++i) {
    VectorXd tmp = estimations[i] - ground_truth[i];
    tmp = tmp.array() * tmp.array();
    rmse = rmse + tmp;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}
