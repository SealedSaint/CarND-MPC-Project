#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10;
double dt = 0.15;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Note: feel free to play around with this or do something completely different
double ref_v = 80 * 0.44704; // mph to m/s since velocity comes in at m/s

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

	// `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
		// The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.

    /* === Reference State Cost === */

		AD<double> cost = 0;

		AD<double> cte_cost_factor = 0.3;
		AD<double> epsi_cost_factor = 30.0;
		AD<double> ev_cost_factor = 1.0;
		AD<double> delta_cost_factor = 50.0;
		AD<double> a_cost_factor = 1.0;
		AD<double> deltaD_cost_factor = 10.0;
		AD<double> aD_cost_factor = 1.0;

    // The part of the cost based on the reference state.
		// consider cte and epsi, plus v/ref_v
		for (int t = 0; t < N; ++t) {
			cost += cte_cost_factor * CppAD::pow(vars[cte_start + t], 2);
			cost += epsi_cost_factor * CppAD::pow(vars[epsi_start + t], 2);
			cost += ev_cost_factor * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

    // Minimize the use of actuators (prefer to not change)
		for (int t = 0; t < N - 1; ++t) {
			cost += delta_cost_factor * CppAD::pow(vars[delta_start + t], 2);
			cost += a_cost_factor * CppAD::pow(vars[a_start + t], 2);
		}

    // Minimize the value gap between sequential actuations (don't change a lot)
		for (int t = 0; t < N - 2; ++t) {
			cost += deltaD_cost_factor * CppAD::pow(vars[delta_start + t] - vars[delta_start + t + 1], 2);
			cost += aD_cost_factor * CppAD::pow(vars[a_start + t] - vars[a_start + t + 1], 2);
		}

		fg[0] = cost;

		// Setup Constraints
    // Note: In this section you'll setup the model constraints.

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
			AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[3]*pow(x0, 3) + coeffs[2]*pow(x0, 2) + coeffs[1]*x0 + coeffs[0]; // Path value at x0
      AD<double> psides0 = CppAD::atan(3.0*coeffs[3]*pow(x0, 2) + 2.0*coeffs[2]*x0 + coeffs[1]); // Desired heading at x0

			// The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The idea here is to constraint this value to be 0.

      // Note the use of `AD<double>` and use of `CppAD`!
      // This is so CppAD can compute derivatives and pass these to the solver.

			// Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			//   # line_val - where_we_are + (additional error from angle error)
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
			//   # heading - desired_heading + (heading_change)

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + (v0 / Lf) * delta0 * dt);
		}
	}
};

/* ==== MPC class definition ==== */

MPC::MPC() {}
MPC::~MPC() {}

vector<vector<double>> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	// number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
	// Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

	// Lower and upper limits for state
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

	// Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
  // Note: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // Note: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

	constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // Note: You don't have to worry about these options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          1.0\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution
	);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;
	vector<double> actuations = {solution.x[delta_start], solution.x[a_start]};
	vector<double> x_vals;
	for(int i = x_start; i < y_start; ++i) {
		x_vals.push_back(solution.x[i]);
	}
	vector<double> y_vals;
	for(int i = y_start; i < psi_start; ++i) {
		y_vals.push_back(solution.x[i]);
	}
	return {actuations, x_vals, y_vals};
}
