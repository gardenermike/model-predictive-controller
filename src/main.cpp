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
          double steering_angle = j[1]["steering_angle"];

          // account for latency
          // use estimated values from the future for our model
          // Since acceleration is typically small, I am assuming constant velocity
          double latency = 1.01e-1; // 100 ms from simulator, plus processing time
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi += v / mpc.Lf * -steering_angle * latency;


          // the points need to be converted to vehicle coordinates
          // to make the math sane
          std::vector<double> vehicle_x;
          std::vector<double> vehicle_y;
          //Eigen::VectorXd y_vector(ptsy.size());
          for (int i = 0; i < ptsx.size(); ++i) {
            double x = ptsx[i];
            double y = ptsy[i];
            double delta_x = x - px;
            double delta_y = y - py;
            vehicle_x.push_back(delta_x * cos(-psi) - delta_y * sin(-psi));
            vehicle_y.push_back(delta_x * sin(-psi) + delta_y * cos(-psi));
          }

          Eigen::VectorXd coeffs = polyfit(
              Eigen::Map<Eigen::VectorXd>(&vehicle_x[0], vehicle_x.size()),
              Eigen::Map<Eigen::VectorXd>(&vehicle_y[0], vehicle_y.size()),
              3);
          //Eigen::VectorXd coeffs = polyfit(x_vector, y_vector, 3);


          double cte = polyeval(coeffs, px) - py;
          double epsi = psi - atan(coeffs[1]);

          Eigen::VectorXd state(6);
          //state << px, py, psi, v, cte, epsi;
          // since we've converted from map to vehicle coordinates,
          // we are at the origin with 0 psi, so the first 3 elements of the state are zero
          state << 0, 0, 0, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

					//std::cout << "x = " << vars[0] << std::endl;
					//std::cout << "y = " << vars[1] << std::endl;
					//std::cout << "psi = " << vars[2] << std::endl;
					//std::cout << "v = " << vars[3] << std::endl;
					//std::cout << "cte = " << vars[4] << std::endl;
					//std::cout << "epsi = " << vars[5] << std::endl;
					//std::cout << "delta = " << vars[6] << std::endl;
					//std::cout << "a = " << vars[7] << std::endl;
					//std::cout << std::endl;


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = vars[0];
          if (steer_value > 1) steer_value = 1;
          if (steer_value < -1) steer_value = -1;
          double throttle_value = vars[1];
          if (throttle_value > 1) throttle_value = 1;
          if (throttle_value < -1) throttle_value = -1;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / (M_PI / 8);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          int points_size = (vars.size() - 2) / 2;
          for (int i = 0; i < points_size; i++) {
            mpc_x_vals.push_back(vars[2 * i + 2]);
            mpc_y_vals.push_back(vars[2 * i + 2 + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = vehicle_x; //next_x_vals;
          msgJson["next_y"] = vehicle_y; //next_y_vals;


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
