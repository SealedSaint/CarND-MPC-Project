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
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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

vector<vector<double>> transformVectors(
	double x1, double y1, double psi,
	vector<double> ptsx, vector<double> ptsy
) {
	assert(ptsx.size() == ptsy.size());
	vector<vector<double>> finals;
	vector<double> newx;
	vector<double> newy;

	for(int i = 0; i < ptsx.size(); ++i) {
		double x2 = ptsx[i];
		double y2 = ptsy[i];
		newx.push_back(cos(psi)*x2 + sin(psi)*y2 - (cos(psi)*x1 + sin(psi)*y1));
		newy.push_back(-sin(psi)*x2 + cos(psi)*y2 + sin(psi)*x1 - cos(psi)*y1);
	}

	finals = {newx, newy};
	return finals;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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

					// Transform ptsx and ptsy into car coordinates
					vector<vector<double>> transformed = transformVectors(px, py, psi, ptsx, ptsy);
					vector<double> ptsx_car = transformed[0];
					vector<double> ptsy_car = transformed[1];

					Eigen::VectorXd eig_ptsx = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx_car.data(), ptsx_car.size());
					Eigen::VectorXd eig_ptsy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy_car.data(), ptsy_car.size());
					auto coeffs = polyfit(eig_ptsx, eig_ptsy, 3);

					double x_car = 0.0;
					double y_car = 0.0;
					double psi_car = 0.0;

					double cte = polyeval(coeffs, x_car) - y_car;

					double derivative = 3.0*coeffs[3]*pow(x_car, 2) + 2.0*coeffs[2]*x_car + coeffs[1];
				  double epsi = psi_car - atan(derivative);

					Eigen::VectorXd state(6);
				  state << x_car, y_car, psi_car, v, cte, epsi;

					// std::cout << "WE ARE HERE (x, y, psi): (" << px << "," << py << "," << psi << ")" << std::endl;
					// std::cout << "NEXT POINTS (x then y):" << std::endl;
					// for (auto const& c : ptsx) std::cout << c << ' '; std::cout << std::endl;
					// for (auto const& c : ptsy) std::cout << c << ' '; std::cout << std::endl;

					// Steering angle and throttle are between [-1, 1]
					vector<vector<double>> vals = mpc.Solve(state, coeffs);
					vector<double> actuations = vals[0];
					vector<double> x_vals = vals[1];
					vector<double> y_vals = vals[2];

					double steer_value = actuations[0];
					double throttle_value = actuations[1];

          json msgJson;
          // Note: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)] instead of [-1, 1].
          msgJson["steering_angle"] = -1.0 * steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
					// Transform ptsx and ptsy into car coordinates
					// transformed = transformVectors(px, py, psi, x_vals, y_vals);
					// vector<double> x_vals_car = transformed[0];
					// vector<double> y_vals_car = transformed[1];

					mpc_x_vals = x_vals;
					mpc_y_vals = y_vals;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
					next_x_vals = ptsx_car;
					next_y_vals = ptsy_car;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value, but should be able to drive
          // around the track with 100ms latency.
          //
          // TODO: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
