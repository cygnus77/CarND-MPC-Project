#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  static Eigen::VectorXd Derivative(const Eigen::VectorXd& coeffs);

  template <typename T>
  static T Eval(const Eigen::VectorXd& coeffs, T& x);

};

#endif /* MPC_H */
