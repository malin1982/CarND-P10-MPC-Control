#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// a bigger time length and step lead to longer trajectory path
// during sharp turns, the fitted trajectory path may become swirling lines
// therefore a proper N & dt should be considered

const size_t N = 20;
const double dt = 0.05; //50ms

const double Lf = 2.67;

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */
