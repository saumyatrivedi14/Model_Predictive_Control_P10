#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double delta = j[1]["steering_angle"];
		  double a = j[1]["throttle"];
		  
		  //Transform Map CS to car CS
          for (unsigned int i=0; i<ptsx.size(); i++){
                double x_shift = ptsx[i] - px;
                double y_shift = ptsy[i] - py;
                ptsx[i] = x_shift * cos(-psi) - y_shift * sin(-psi);
                ptsy[i] = x_shift * sin(-psi) + y_shift * cos(-psi);
          }

          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_car(ptrx, 6);

          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_car(ptry, 6);

          /*
          * Calculate steering angle and throttle using MPC.
		  * Considering simulator latency of 100 ms, based on this next states are predicted
		  */
		  
		  // Fitting a polynomial to the above calculated x & y co-ordinates
		  auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
		  
		  // To Calculate CTE, 
		  // x & y should be kept zero as points are already tranformed to car CS
		  // Otherwise 'y' would be subtracted from polyeval value
		  double cte = polyeval(coeffs, 0);
		  
		  // For orientation error, as x = 0 for car CS, only coeffs[1] will be considered.
		  double epsi = -atan(coeffs[1]);
		  
		  // Predicting states after latency
		  double latency = 0.1;
		  
		  double px_pred = 0.0 + v*latency; // psi is zero hence cos(0) is ignored
		  double py_pred = 0.0; // psi is zero hence v*sin(0)*latency = 0
		  double psi_pred = 0.0 + v * (-delta/Lf) * latency;
		  double v_pred = v + a*latency;
		  double cte_pred = cte + v*sin(epsi)*latency;
		  double epsi_pred = epsi + v*(-delta/Lf) * latency;
		  
		  // Pushing all states to a vector to feed MPC::Solve() function
		  Eigen::VectorXd curr_state(6);
		  curr_state << px_pred, py_pred, psi_pred, v_pred, cte_pred, epsi_pred;
		  
		  // Using MPC::Solve() to find steer_Value & throttle_value
		  vector<double> optimal_output;
		  optimal_output = mpc.Solve(curr_state, coeffs);
		  
		  // Steering value must be divided by deg2rad(25) to normilize it		  
          double steer_value = optimal_output[0] / (deg2rad(25)*Lf);
          double throttle_value = optimal_output[1];

          json msgJson;
          
		  msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = {curr_state[0]};
          vector<double> mpc_y_vals = {curr_state[1]};
		  
		  for (unsigned int i=2; i<optimal_output.size(); i++){
			if((i%2) == 0){
				mpc_x_vals.push_back(optimal_output[i]);
			}
			else{
				mpc_y_vals.push_back(optimal_output[i]);
			}
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  for (unsigned int i=0; i<50; i+=3){
			next_x_vals.push_back(i);
			next_y_vals.push_back(polyeval(coeffs, i));
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
		  

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
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
