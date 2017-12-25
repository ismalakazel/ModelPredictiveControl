#ifndef VEHICLE_H 
#define VEHICLE_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"


using namespace std;
using namespace Eigen;

/// An MPC describable object
class Vehicle {
    public:

        /// The vehicle's x coordinate
        double x;

        /// The vehicle's y coordinate
        double y;

        /// The vehicle's orientation
        double orientation;

        /// The vehicle's speed
        double speed;

        /// The vehicle's steering angle
        double steering;

        /// The vehicle's throttle
        double throttle;

        /**!
          Converts global location (x, y) coordinates to vehicle (x, y) coordinates
          @param xp   A set of x coordinate points
          @param yp   A set of y coordinate points
          @return An array of x and y coverted points
         */
        vector<VectorXd> convertCoordinates(vector<double> xp, vector<double> yp); 

        /**!
          Moves the vehicle
          @param cte  The cross track error of the steering angle
          @param epsi The throttle throttle error
         */
        void move(double cte, double epsi);

        /**!
        Builds polymonial from x and y coordinate points
        @param xp   A vector with x coordinate
        @param yp   A vector with y coordinate
        @param order    The order of the polynomial
        @returns    VectorXd coordinate of x and y 
        */
        Eigen::VectorXd polyfit(Eigen::VectorXd xp, Eigen::VectorXd yp, int order);

        /**!
        Evaluates the polynomial
        @param coeffs   A VectorXd of (x, y) coordinates
        @param x    The x coordinate for comparison
        */
        double polyeval(Eigen::VectorXd coeffs, double x);

        /**!
        Builds a route using the MPC class
        @param coefficients   A VectorXd of (x, y) coordinates
        @returns  A tuple of x and y vectors
        */
        tuple<vector<double>, vector<double>> build_route(VectorXd coefficients);

        /**!
        Builds a trajectory from a VectorXd of x and y coordinates
        @param coefficients    A VectorXd of (x, y) coordinates
        @param N    How long the trajectory should be
        @param step     The interval between each point in the trajectory    
        */
        tuple<vector<double>, vector<double>> build_trajectory(Eigen::VectorXd coefficients, int N, double step);

        /// Retrives the current state of the vehicle
        VectorXd state();

    private:

        /// The Model Predictive Control class
        MPC mpc;

        /// Distance between front of the vehicle and its center of gravity
        const double Lf = 2.67;

        /// Time delay for next state prediction 
        const double dt = 0.1; 
};

#endif /* VEHICLE_H*/

