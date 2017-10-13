#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the number of time steps and the duration of one time step
const size_t N = 10;
const double dt = 0.5;

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

const double ref_v = 30.0 * 0.44704; // The target speed in meters per second.

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should establish
// when one variable starts and another ends to make our lives easier.
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


  void operator()(ADvector& fg, const ADvector& vars)
  {
	    size_t t = 0;

	    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		fg[0] = 0;

		// Reference State Cost
		for (t = 0; t < N; t++) {
		  fg[0] += 1000 * CppAD::pow(vars[cte_start + t], 2);
		  fg[0] += 20000 * CppAD::pow(vars[epsi_start + t], 2);
		  // The target velocity at time step t depends on the steering angle at time step t.
		  // The minimum velocity is 30.0 mph.
		  //AD<double> target_v = 30.0 + 50.0 * (1.0 - fabs(vars[delta_start + t]));
		  fg[0] += 1000 * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Actuator use cost, i.e. minimize the use of actuators for a smoother drive
		for (t = 0; t < N - 1; t++) {
		  fg[0] += 50000 * CppAD::pow(vars[delta_start + t], 2);
		  fg[0] += 1000 * CppAD::pow(vars[a_start + t], 2);
		}

		// Actuator change rate cost, i.e. minimize the change in actuator use for a smoother drive
		for (t = 0; t < N - 2; t++) {
		  fg[0] += 40000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
		  fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		// Set up the constraints

		// Initial constraints

		// We add 1 to each of the starting indices due to the cost being located at
		// index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[x_start + 1] = vars[x_start];
		fg[y_start + 1] = vars[y_start];
		fg[psi_start + 1] = vars[psi_start];
		fg[v_start + 1] = vars[v_start];
		fg[cte_start + 1] = vars[cte_start];
		fg[epsi_start + 1] = vars[epsi_start];

		// The rest of the constraints
		for (t = 0; t < N - 1; t++)
		{

		/* Note the use of `AD<double>` and use of `CppAD`!
		 CppAD can compute derivatives and pass
		 these to the solver.
		 State at time t */
		AD<double> x0 = vars[x_start + t];
		AD<double> y0 = vars[y_start + t];
		AD<double> psi0 = vars[psi_start + t];
		AD<double> v0 = vars[v_start + t];
		AD<double> cte0 = vars[cte_start + t];
		AD<double> epsi0 = vars[epsi_start + t];

		// State at time t + 1
		AD<double> x1 = vars[x_start + t + 1];
		AD<double> y1 = vars[y_start + t + 1];
		AD<double> psi1 = vars[psi_start + t + 1];
		AD<double> v1 = vars[v_start + t + 1];
		AD<double> cte1 = vars[cte_start + t + 1];
		AD<double> epsi1 = vars[epsi_start + t + 1];

		// Actuation at time t
		AD<double> delta0 = vars[delta_start + t];
		AD<double> a0 = vars[a_start + t];
		// Reference trajectory evaluated at x_t
		AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
		// Derivative of the reference trajectory evaluated at x_t
		AD<double> df0 = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2);
		AD<double> psides0 = CppAD::atan(df0);

		/*  The equations of the kinematic vehicle model being used here:
			x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			v_[t+1] = v[t] + a[t] * dt
			cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

			Formulate all model constraints to take the form c - g(x) = 0.
		*/

		fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
		fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
		fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {};
MPC::~MPC() {};

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	//size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	// Number of independent variables
	// N timesteps == N - 1 actuations
	size_t n_vars = N * 6 + (N - 1) * 2;
	// Number of constraints
	size_t n_constraints = N * 6;

	size_t i = 0;

	// Initial value of the independent variables.
	// Should be 0 besides initial state.
	Dvector ivars(n_vars);
	for (i = 0; i < n_vars; i++)
	{
		ivars[i] = 0;
	}
	// Set the initial variable values
	ivars[x_start] = x;
	ivars[y_start] = y;
	ivars[psi_start] = psi;
	ivars[v_start] = v;
	ivars[cte_start] = cte;
	ivars[epsi_start] = epsi;

	Dvector ivars_lowerbound(n_vars);
	Dvector ivars_upperbound(n_vars);

	// Set all non-actuators upper and lower limits
	// to the max negative and positive values.
	for (i = 0; i < delta_start; i++)
	{
	ivars_lowerbound[i] = -1.0e19;
	ivars_upperbound[i] = 1.0e19;
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	for (i = delta_start; i < a_start; i++)
	{
	ivars_lowerbound[i] = -0.436332;
	ivars_upperbound[i] = 0.436332;
	}

	// Acceleration/deceleration upper and lower limits.
	for (i = a_start; i < n_vars; i++)
	{
	ivars_lowerbound[i] = -1.0;
	ivars_upperbound[i] = 1.0;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector cnstr_lowerbound(n_constraints);
	Dvector cnstr_upperbound(n_constraints);

	for (i = 0; i < n_constraints; i++)
	{
	  cnstr_lowerbound[i] = 0;
	  cnstr_upperbound[i] = 0;
	}
	cnstr_lowerbound[x_start] = x;
	cnstr_lowerbound[y_start] = y;
	cnstr_lowerbound[psi_start] = psi;
	cnstr_lowerbound[v_start] = v;
	cnstr_lowerbound[cte_start] = cte;
	cnstr_lowerbound[epsi_start] = epsi;

	cnstr_upperbound[x_start] = x;
	cnstr_upperbound[y_start] = y;
	cnstr_upperbound[psi_start] = psi;
	cnstr_upperbound[v_start] = v;
	cnstr_upperbound[cte_start] = cte;
	cnstr_upperbound[epsi_start] = epsi;

	// Object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// Options for IPOPT solver
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
	options += "Numeric max_cpu_time          0.5\n";

	// Place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// Solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
	  options, ivars, ivars_lowerbound, ivars_upperbound, cnstr_lowerbound,
	  cnstr_upperbound, fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;
	std::cout << "Cost: " << cost << std::endl;

	// Convert the solution to an STL vector.
	vector<double> output;
	output.push_back(solution.x[delta_start]);
	output.push_back(solution.x[a_start]);

	for (i = 1; i < N; i++)
	{
	output.push_back(solution.x[x_start + i]);
	output.push_back(solution.x[y_start + i]);
	}

	return output;

}
