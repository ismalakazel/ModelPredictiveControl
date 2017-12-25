#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "Vehicle.h"


// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  Vehicle vehicle;

  h.onMessage([&vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        
        // JSON object containing state and control variables
        auto JSON = json::parse(s);
        
        // Telemetry event
        string event = JSON[0].get<string>();
        
        if (event == "telemetry") {
          
          // Points on the x axis
          vector<double> ptsx = JSON[1]["ptsx"];

          // Points on the y axis
          vector<double> ptsy = JSON[1]["ptsy"];
          
          // Update vehicle variables
          vehicle.x = JSON[1]["x"];
          vehicle.y = JSON[1]["y"];
          vehicle.orientation = JSON[1]["psi"];
          vehicle.speed = JSON[1]["speed"];
          vehicle.steering = JSON[1]["steering_angle"];
          vehicle.throttle = JSON[1]["throttle"];

          // Update vehicle (x, y) coordinates from global (x, y) coordinates
          vector<VectorXd> coordinates = vehicle.convertCoordinates(ptsx, ptsy); 

          // Fit a 3rd order polynomial line for x and y points
          Eigen::VectorXd coefficients = vehicle.polyfit(coordinates[0], coordinates[1], 3);

          // Compute cross track error
          double cte = vehicle.polyeval(coefficients, 0);

          // Compute throttle error
          double epsi = -atan(coefficients[1]);
         
          // Predict next vehicle state 
          vehicle.move(cte, epsi);

          /// Build route (green lline)
          auto route = vehicle.build_route(coefficients);

          /// Build trajectory (yellow line)
          auto trajectory = vehicle.build_trajectory(coefficients, 20, 3);

          json msgJson;
          
          msgJson["steering_angle"] = vehicle.steering;
          msgJson["throttle"] = vehicle.throttle;
    
          msgJson["mpc_x"] = get<0>(route);
          msgJson["mpc_y"] = get<1>(route);
 
          msgJson["next_x"] = get<0>(trajectory);
          msgJson["next_y"] = get<1>(trajectory);

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
