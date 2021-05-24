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

const double MIN_TOLERANCE = 0.002; // Only used when twiddle is enabled
const bool TWIDDLE_ENABLED = false; // Enable it to keep optimizing PID's params
const int AMOUNT_OF_ITERATIONS = 10000; // Used when twiddle is enabled
const bool FAST_AND_FURIOUS_MODE_ENABLED = false; // Enable it to go FAST!
const double MAX_CTE = 0.3; // Used to handle throttle and steering in a different way

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

double calculateThrottle(double cte, double speed, double total_error) {
  if (cte > MAX_CTE) {
    // Special case where we prefer to avoid using the PID values in favor of
    // reducing the speed when the error is big enough.
    if (speed > 30) {
      return 0;
    } else {
      return 0.2;
    }
  }
  if (!FAST_AND_FURIOUS_MODE_ENABLED) {
    return 0.3;
  }
  // PID was configured to return a bigger negative values when error is bigger
  // With this we assure the value is between 0.3 and 1 (when total_error is 0)
  return 1 + max(-0.7, total_error);
}

int main() {
  uWS::Hub h;

  PID steeringPID;

  double Kp = 0.225698;
  double Ki = 0.000216;
  double Kd = 9.644250;

  steeringPID.Init(Kp, Ki, Kd);
  std::vector<double> params{ Kp, Ki, Kd };
  ParamsOptimizer paramsOptimizer{ params, MIN_TOLERANCE};

  PID throttlePID;
  throttlePID.Init(-1.5, 0, 0.5); //Params manually optimized

  int count = 0;
  double total_error = 0;

  h.onMessage([
    &steeringPID,
    &throttlePID,
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
          double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());

          steeringPID.UpdateError(-cte);
          double steer_value = steeringPID.TotalError();

          if (TWIDDLE_ENABLED) {
            if (count > AMOUNT_OF_ITERATIONS) {
              std::vector<double> new_params = paramsOptimizer.getParams(total_error / count);
              steeringPID.Init(new_params[0], new_params[1], new_params[2]);
              count = 0;
              total_error = 0;
            } else {
              total_error += fabs(cte);
              count++;
            }
          } else {
            if (count > AMOUNT_OF_ITERATIONS) {
              std::cout << "Average error: " << total_error / count << std::endl;
              count = 0;
              total_error = 0;
            } else {
              total_error += fabs(cte);
              count++;
            }
          }

          throttlePID.UpdateError(fabs(cte));
          double throttle = calculateThrottle(fabs(cte), speed, throttlePID.TotalError());

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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