#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "params_optimizer.h"

// for convenience
using nlohmann::json;
using std::string;
using std::max;
using std::min;

const double MIN_TOLERANCE = 0.002;
const bool TWIDDLE_ENABLED = false;
const int AMOUNT_OF_ITERATIONS = 10000;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// double calculateThrottle(double cte) {
//   double possitive_cte = fabs(cte);
//   if (fabs(cte) < 0.01 ) {
//     return 1.1 - (possitive_cte / 0.01);
//   }
//   return 0.3;
// }

int main() {
  uWS::Hub h;

  PID pid;

  double Kp = 0.08; 
  double Ki = 0.000166;
  double Kd = 2.501;

  pid.Init(Kp, Ki, Kd);
  std::vector<double> params{ Kp, Ki, Kd };
  ParamsOptimizer paramsOptimizer{ params, MIN_TOLERANCE};

  int count = 0;
  double total_error = 0;

  h.onMessage([
    &pid,
    &paramsOptimizer,
    &count,
    &total_error
  ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          // double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());

          pid.UpdateError(-cte);
          double steer_value = pid.TotalError();

          if (TWIDDLE_ENABLED) {
            if (count > AMOUNT_OF_ITERATIONS) {
              std::vector<double> new_params = paramsOptimizer.getParams(total_error / count);
              pid.Init(new_params[0], new_params[1], new_params[2]);
              count = 0;
              total_error = 0;
            } else {
              total_error += fabs(cte);
              count++;
            }
          }

          // double throttle = calculateThrottle(cte);

          // DEBUG
          // std::cout << "cte: " << cte << " steer_value: " << steer_value << std::endl;
          // std::cout << "cte: " << cte << " throttle: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, 
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