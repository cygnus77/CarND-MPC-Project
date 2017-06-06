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

	static Eigen::VectorXd Derivative(const Eigen::VectorXd& coeffs)
	{
	  Eigen::VectorXd result(coeffs.rows()-1);
	  for(int i = 1; i < coeffs.rows(); i++) {
	    result[i-1] = i * coeffs[i];
	  }
	  return result;
	}

	template <typename T>
	static T Eval(const Eigen::VectorXd& coeffs, const T& x)
	{
	  T result = 0;
	  for(int i = 0 ; i < coeffs.rows(); i++) {
	    result += (coeffs[i] * pow(x, i));
	  }
	  return result;
	}

};

#endif /* MPC_H */
