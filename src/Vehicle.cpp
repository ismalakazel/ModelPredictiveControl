#include "Vehicle.h"


// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


vector<VectorXd> Vehicle::convertCoordinates(vector<double> xp, vector<double> yp) {

    // Verify x and y sizes
    if (xp.size() != yp.size()) {
        std::exception();
    }

    // Points to vehicle x and y coordinates placeholder vectors
    VectorXd x_values(xp.size());
    VectorXd y_values(yp.size());

    // Convert global coordinates do vehicle coordinates
    for (int n = 0; n < xp.size(); n++) {
        double x = xp[n] - this->x;
        double y = yp[n] - this->y;
        x_values[n] = x * cos(-orientation) - y * sin(-orientation);
        y_values[n] = x * sin(-orientation) + y * cos(-orientation);
    }

    // Array of converted x and y coordinates
    vector<VectorXd> result;
    result.push_back(x_values);
    result.push_back(y_values);

    return result;
};


void Vehicle::move(double cte, double epsi) {

    // Predict state variables
    x = speed * dt;
    y = 0.0;
    orientation = speed * -steering / Lf * dt;
    speed += throttle * dt;

    // Predict control variables
    this->cte = cte + speed * sin(epsi) * dt;
    this->epsi = epsi + speed * -steering / Lf * dt;        
};


tuple<vector<double>, vector<double>> Vehicle::build_route(VectorXd coefficients) {
    vector<double> mpc_route = mpc.Solve(this->state(), coefficients); 
    this->cte = mpc_route[0] / (deg2rad(25) * Lf);
    this->epsi = mpc_route[1];

    vector<double> mpc_x_vals = {x};
    vector<double> mpc_y_vals = {y};

    for (int i = 2; i < mpc_route.size(); i++) {
        if (i % 2 == 0) {
            mpc_x_vals.push_back(mpc_route[i]);
        } else {
            mpc_y_vals.push_back(mpc_route[i]);
        }
    }

    return make_tuple(mpc_x_vals, mpc_y_vals); 
};


tuple<vector<double>, vector<double>> Vehicle::build_trajectory(Eigen::VectorXd coefficients, int N, double step) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 2; i < N; i++) {
        next_x_vals.push_back(step * i);
        next_y_vals.push_back(polyeval(coefficients, step * i));
    }

    return make_tuple(next_x_vals, next_y_vals);
};


VectorXd Vehicle::state() {
    VectorXd state(6);
    state << x, y, orientation, speed, cte, epsi;
    return state;
};


double Vehicle::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

Eigen::VectorXd Vehicle::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

