#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <limits>
#include <algorithm>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

int main()
{
  uWS::Hub h;
  
  const int REQ_CTE_OBSERVATIONS = 1800;

  PID pid(REQ_CTE_OBSERVATIONS, 0.05);
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          double steer_value = clip(pid.TotalError(), -1, 1);
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  auto handle = std::async([&]{h.run();});
  
  std::vector<double> twiddle_pid  { 0.211576,  0.0005, 2.57 };
  std::vector<double> twiddle_dpid { 0.05,      1e-4,   0.2  };

  double twiddle_dpid_sum;
  double best_error = std::numeric_limits<double>::max(), error;

  do {
    
    for (int i = 0; i < 3; i++) {
      twiddle_pid[i] += twiddle_dpid[i];
      error = pid.get_i_error_l2_with_params(twiddle_pid[0], twiddle_pid[1], twiddle_pid[2]);
      if (error < best_error) {
        best_error = error;
        twiddle_dpid[i] *= 1.1;
      } else {
        twiddle_pid[i] -= 2 * twiddle_dpid[i];
        if (i != 2)
          twiddle_pid[i] = std::max<double>(twiddle_pid[i], 0);
        error = pid.get_i_error_l2_with_params(twiddle_pid[0], twiddle_pid[1], twiddle_pid[2]);
        if (error < best_error) {
          best_error = error;
          twiddle_dpid[i] *= 1.1;
        } else {
          twiddle_pid[i] += twiddle_dpid[i];
          twiddle_dpid[i] *= 0.9;
        }
      }
      std::cout << "dpid " << twiddle_dpid[0] << " " << twiddle_dpid[1] << " "<< twiddle_dpid[2] << std::endl;
    }
    
    twiddle_dpid_sum = twiddle_dpid[0] + twiddle_dpid[1] + twiddle_dpid[2];
    
  } while (twiddle_dpid_sum > 0.001);
  
  std::cout << "Out !" << std::endl;

}
