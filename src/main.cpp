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

  // initialize the controller
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
          double steering_angle = j[1]["steering_angle"];

          // account for latency
          // use estimated values from the future for our model rather than the current values
          // Since acceleration is typically small, I am assuming constant velocity
          double latency = 1.01e-1; // 100 ms from simulator, plus a small amount of processing time
          px += v * cos(psi) * latency; // the x value in latency seconds, assuming constant velocity
          py += v * sin(psi) * latency; // the y value in latency seconds, assuming constant velocity
          psi += v / mpc.Lf * -steering_angle * latency; // the bearing in latency seconds, assuming a constant steering angle


          // the points need to be converted to vehicle coordinates
          // to make the math sane.
          // when everything is in vehicle coordinates, then
          // we are positioned at the origin, oriented directly forward,
          // so x, y, and psi are all zero
          std::vector<double> vehicle_x;
          std::vector<double> vehicle_y;
          for (int i = 0; i < ptsx.size(); ++i) {
            double x = ptsx[i];
            double y = ptsy[i];
            double delta_x = x - px;
            double delta_y = y - py;
            vehicle_x.push_back(delta_x * cos(-psi) - delta_y * sin(-psi));
            vehicle_y.push_back(delta_x * sin(-psi) + delta_y * cos(-psi));
          }

          // Fit a degree 3 polynomial to the observed waypoints
          Eigen::VectorXd coeffs = polyfit(
              Eigen::Map<Eigen::VectorXd>(&vehicle_x[0], vehicle_x.size()),
              Eigen::Map<Eigen::VectorXd>(&vehicle_y[0], vehicle_y.size()),
              3);

          // Find our cross track error and orientation error
          // against the calculated polynomial representing the center line
          double cte = polyeval(coeffs, px) - py;
          double epsi = psi - atan(coeffs[1]);

          Eigen::VectorXd state(6);
          //state << px, py, psi, v, cte, epsi; // we don't need this complexity :)

          // since we've converted from map to vehicle coordinates,
          // we are at the origin, oriented on the x-axis, so the first 3 elements of the state are zero
          state << 0, 0, 0, v, cte, epsi;

          /*
          * Calculate steering angle and throttle using the MPC.
          *
          * Steering angle and throttle are both in [-1, 1].
          *
          */
          auto vars = mpc.Solve(state, coeffs);
          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // Divide by maximum angle set in MPC before sending the steering value back.
          // Otherwise the values will be constrained by that value instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / (M_PI / 8);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          int points_size = (vars.size() - 2) / 2;
          for (int i = 0; i < points_size; i++) {
            mpc_x_vals.push_back(vars[2 * i + 2]);
            mpc_y_vals.push_back(vars[2 * i + 2 + 1]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          // display the waypoints
          msgJson["next_x"] = vehicle_x;
          msgJson["next_y"] = vehicle_y;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
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
