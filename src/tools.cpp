#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() ==0)
        return rmse;
  /**
  TODO:
    * Calculate the RMSE here.
  */
  for (int i = 0; i<estimations.size(); i++){
      VectorXd diff (4);
      diff = estimations[i]-ground_truth[i];
      diff = diff.array().pow(2);
      rmse += diff;
  }
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  cout << "rmse :" << rmse(0) << "|" << rmse (1) << "|" << rmse(2) << "|" << rmse(3) << endl;

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
