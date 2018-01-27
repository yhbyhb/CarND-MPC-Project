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

using namespace std::chrono;
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

Eigen::MatrixXd transformCoordiate(double x, double y, double psi,
  const vector<double>& ptsx, const vector<double>& ptsy) {
  const int numPoints = ptsx.size();

  Eigen::MatrixXd targetPoints = Eigen::MatrixXd(2, numPoints);
  for (int i = 0; i < numPoints; ++i)
  {
      const double diff_x = ptsx[i] - x;
      const double diff_y = ptsy[i] - y;
      targetPoints(0, i) = cos(psi) * diff_x - sin(psi) * diff_y;
      targetPoints(1, i) = sin(psi) * diff_x + cos(psi) * diff_y;
  }

  return targetPoints;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  int actual_latency_ms = 0;

  h.onMessage([&mpc, &actual_latency_ms](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // measure actual latency
          steady_clock::time_point begin = steady_clock::now();

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // The sign of angle of simulator is opposite to our model (counter clock wise is positive)
          delta = delta * -1;
          psi = psi * -1;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Transform coordination from global to view of car.
          // So, first point is (0, 0) with angle 0
          Eigen::MatrixXd car_pts = transformCoordiate(px, py, psi, ptsx, ptsy);
          Eigen::VectorXd car_ptsx = car_pts.row(0);
          Eigen::VectorXd car_ptsy = car_pts.row(1); 

          // Get third order polynomial coeffs from way points.
          auto coeffs = polyfit(car_ptsx, car_ptsy, 3);

          // mph -> m/s
          v *= 0.447;

          // vehicle's position and its orientation are all 0 after coordinate transformation.  
          px = 0;
          py = 0;
          psi = 0;

          // Latency changes state of car.
          double latency = 0.1;
          if (actual_latency_ms > 100)
          {
            latency = (double)actual_latency_ms / 1000;
            std::cout << "previous latency(ms) : " << latency * 1000 << std::endl; 
          }

          const double Lf = 2.67;
          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;
          psi = psi + v * delta / Lf * latency;
          v = v + a * latency;
          
          double cte = polyeval(coeffs, px) - py;
          double epsi = psi - atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px);
          
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / 0.436332; //deg2rad(25) is 0.436322
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          int n = (vars.size() - 2) / 2;
          for (int i = 0; i < n; ++i)
          {
            mpc_x_vals.push_back(vars[2 + i]);
          }
          for (int i = 0; i < n; ++i)
          {
            mpc_y_vals.push_back(vars[2 + n + i]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          const int numPoints = car_ptsx.size();
          for (int i = 0; i <= car_ptsx[numPoints - 1]; ++i)
          {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // std::cout << msg << std::endl;
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
          steady_clock::time_point end = steady_clock::now();
          actual_latency_ms = duration_cast<milliseconds>(end - begin).count();
          std::cout << "actual latency(ms) : " << actual_latency_ms << std::endl << std::endl;
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
