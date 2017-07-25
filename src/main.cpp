#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include "dlib/optimization/find_optimal_parameters.h"

typedef dlib::matrix<double,0,1> column_vector;

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

PID * optimize_pid;

double get_i_error_l2_pid(column_vector params) {
  double Kp, Ti, Td, Ki, Kd;
  Kp = params(0);
  Ti = params(1);
  Td = params(2);
  Ki = Kp / Ti;
  Kd = Kp * Td;
  std::cout << "trying with " << Kp << " " << Ki << " " << Kd << std::endl;
  double l2 = optimize_pid->get_i_error_l2_with_params(Kp, Ki, Kd);
  std::cout << "got error " << l2 << std::endl;
  return l2;

}

int main()
{
  uWS::Hub h;
  std::mutex lock;
  
  const int REQ_CTE_OBSERVATIONS = 1000;

  PID pid(REQ_CTE_OBSERVATIONS, lock);
  // TODO: Initialize the pid variable.
  //pid.Init(0.4, 0.001, 0.2);

  h.onMessage([&pid, &lock](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

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
  
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000*10));
  
  column_vector pid_params(3), pid_params_min(3), pid_params_max(3);
  
  pid_params_min = 0,    1,                          0;
  pid_params     = 0.15, REQ_CTE_OBSERVATIONS/2,    50;
  pid_params_max = 0.3,  REQ_CTE_OBSERVATIONS,     100;

  while (true) {
  
    //std::cout << "CTE L2 " << pid.get_i_error_l2() << std::endl;;
    //pid.Init(0.4, 0.001, 0.2);
    
    optimize_pid = & pid;
    
    double cte2 = dlib::find_optimal_parameters (
                                    1,
                                    1e-2,
                                    1e3,
                                    pid_params,
                                    pid_params_min,
                                    pid_params_max,
                                    get_i_error_l2_pid
                                    );
    
    std::cout << cte2 << " with params " << pid_params << std::endl;

    //pid.Init(0., 0, 0.);

  }
  //h.run();
}
