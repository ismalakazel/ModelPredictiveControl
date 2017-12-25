#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Number of timesteps in the horizon. 
size_t N = 10;

// Elapsed time between actuations
double dt = 0.1;

const double Lf = 2.67;
const double ref_v = 100;

const int x_start = 0;
const int y_start = x_start + N;
const int psi_start = y_start + N;
const int v_start = psi_start + N;
const int cte_start = v_start + N;
const int epsi_start = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start = delta_start + N - 1;

class FG_eval {
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;
        FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector& fg, const ADvector& vars) {
            
            // Set cost to 0
            fg[0] = 0;

            // The part of the cost based on the reference state.
            for( int i = 0; i < N; i++ ) {
                fg[0] += 1500 * CppAD::pow(vars[cte_start + i], 2);
                fg[0] += 1500 * CppAD::pow(vars[epsi_start + i], 2);
                fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
            }

            // Minimize the use of actuators.
            for (int i = 0; i < N - 1; i++) {
                fg[0] += CppAD::pow(vars[delta_start + i], 2);
                fg[0] += 100 * CppAD::pow(vars[a_start + i], 2);
            }

            // Minimize the value gap between sequential actuations.
            for (int i = 0; i < N - 2; i++) {
                fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
                fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
            }

            // Initial constraints.
            fg[x_start + 1] = vars[x_start];
            fg[y_start + 1] = vars[y_start];
            fg[psi_start + 1] = vars[psi_start];
            fg[v_start + 1] = vars[v_start];
            fg[cte_start + 1] = vars[cte_start];
            fg[epsi_start + 1] = vars[epsi_start];

            for (int t = 1; t < N; t++) {
                
                // State at time t.
                AD<double> x = vars[x_start + t - 1];
                AD<double> y = vars[y_start + t - 1];
                AD<double> psi = vars[psi_start + t - 1];
                AD<double> v = vars[v_start + t - 1];
                AD<double> cte = vars[cte_start + t - 1];
                AD<double> epsi = vars[epsi_start + t - 1];
                
                // Only consider the actuation at time T
                AD<double> delta = vars[delta_start + t - 1];
                AD<double> a = vars[a_start + t - 1];
                AD<double> f = coeffs[0] + coeffs[1] * x + coeffs[2] * CppAD::pow(x, 2) + coeffs[3] * CppAD::pow(x, 3);
                AD<double> psides = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * CppAD::pow(x, 2));

                // State at time T+1
                AD<double> xt = vars[x_start + t];
                AD<double> yt = vars[y_start + t];
                AD<double> psit = vars[psi_start + t];
                AD<double> vt = vars[v_start + t];
                AD<double> ctet = vars[cte_start + t];
                AD<double> epsit = vars[epsi_start + t];
  
                fg[1 + x_start + t] = xt - (x + v * CppAD::cos(psi) * dt);
                fg[1 + y_start + t] = yt - (y + v * CppAD::sin(psi) * dt);
                fg[1 + psi_start + t] = psit - (psi - v / Lf * delta * dt);
                fg[1 + v_start + t] = vt - (v + a * dt);
                fg[1 + cte_start + t] = ctet - ((f - y) + (v * CppAD::sin(epsi) * dt));
                fg[1 + epsi_start + t] = epsit - ((psi - psides) - v / Lf * delta * dt);
            }
        }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    
    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    const size_t n_vars = N * 6 + (N - 1) * 2;
    const size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lower limits
    // to the max negative and positive values.
    for ( int i = 0; i < delta_start; i++ ) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 to 25
    // degrees (values in radians).
    for ( int i = delta_start; i < a_start; i++ ) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.43632;
    }

    // Actuator limits.
    for ( int i = a_start; i < n_vars; i++ ) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector lower_contrains(n_constraints);
    Dvector upper_constrains(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        lower_contrains[i] = 0;
        upper_constrains[i] = 0;
    }

    lower_contrains[x_start] = x;
    lower_contrains[y_start] = y;
    lower_contrains[psi_start] = psi;
    lower_contrains[v_start] = v;
    lower_contrains[cte_start] = cte;
    lower_contrains[epsi_start] = epsi;

    upper_constrains[x_start] = x;
    upper_constrains[y_start] = y;
    upper_constrains[psi_start] = psi;
    upper_constrains[v_start] = v;
    upper_constrains[cte_start] = cte;
    upper_constrains[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    std::string options;
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, lower_contrains,
            upper_constrains, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    vector<double> result;

    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for ( int i = 0; i < N - 2; i++ ) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    return result;
}
