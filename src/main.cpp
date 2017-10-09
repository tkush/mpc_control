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
			cout << sdata << endl << endl;
			if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') 
			{
				string s = hasData(sdata);
				if (s != "") 
				{
					auto j = json::parse(s);
					string event = j[0].get<string>();
					if (event == "telemetry") 
					{
						const double Lf = 2.67;

						// j[1] is the data JSON object
						vector<double> ptsx = j[1]["ptsx"];
						vector<double> ptsy = j[1]["ptsy"];
						double px = j[1]["x"];
						double py = j[1]["y"];
						double psi = j[1]["psi"];
						double v = j[1]["speed"];
						double steer_value = j[1]["steering_angle"];
						double throttle_value = j[1]["throttle"];

						//Convert the track coords to the vehicle frame:
						vector<double> ptsx_car;
						vector<double> ptsy_car;
						
						//Apply latency
						const double latency = 0.1; // 100 milliseconds
						double px_delay = px + v*cos( psi )*latency;
						double py_delay = py + v*sin( psi )*latency;
						double psi_delay = psi + v * steer_value / Lf * latency;
						double v_delay = v + throttle_value * latency;
						
						for ( int i=0; i<ptsx.size(); i++)
						{
							//Translate track origin to car origin						
							double x_transl = ptsx[i] - px_delay;
							double y_transl = ptsy[i] - py_delay;

							//Rotate the map coord system to car coord system
							//Essentially rotate CCW by psi
							// | x_rot | = |  cos(psi)	 sin(psi) | *  | x |  
							// | y_rot | = | -sin(psi)	 cos(psi) |	   | y |   
							double x_car =  x_transl * cos(psi_delay) + y_transl * sin(psi_delay);
							double y_car = -x_transl * sin(psi_delay) + y_transl * cos(psi_delay);

							ptsx_car.push_back(x_car);
							ptsy_car.push_back(y_car);
						}

						//Convert track points to eigen vector
						Eigen::VectorXd ptsx_curve(ptsx_car.size());
						Eigen::VectorXd ptsy_curve(ptsy_car.size());
						for (int i=0; i<ptsx_car.size(); i++)
						{
							ptsx_curve(i) = ptsx_car[i];
							ptsy_curve(i) = ptsy_car[i];
						}

						//Fit a third order polynomial through these points
						auto coeffs = polyfit(ptsx_curve, ptsy_curve, 3);

						//Calculate CTE at car's current position (0,0)
						double cte = polyeval(coeffs, 0);

						//Calculate first derivative of the third order polynomial
						// f = a_0 + a_1*x + a_2*x^2 + a_3*x^3
						// f'=       a_1   + 2*a_2*x + 3*a_3*x^2
						double f_dot = coeffs[1];

						//Tangent to the curve is f'. Psi is the arctan of this
						double epsi = - atan(f_dot);

						////Define state vector for optimization
						Eigen::VectorXd state(6);
						state << 0., 0. , 0, v_delay, cte, epsi;
						//state << 0, 0 , 0, v, cte, epsi;
						auto vars = mpc.Solve(state, coeffs);
						//cout << "Here" << endl;

						json msgJson;
						// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
						// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
						msgJson["steering_angle"] = -vars[0]/deg2rad(25);
						msgJson["throttle"] = vars[1];

						//Display the MPC predicted trajectory 
						vector<double> mpc_x_vals;
						vector<double> mpc_y_vals;

						for ( int i=2; i<vars.size(); i++)
						{
							if ( i%2 == 0 )
								mpc_x_vals.push_back(vars[i]);
							else
								mpc_y_vals.push_back(vars[i]);
						}

						//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
						// the points in the simulator are connected by a Green line
						msgJson["mpc_x"] = mpc_x_vals;
						msgJson["mpc_y"] = mpc_y_vals;

						

						//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
						// the points in the simulator are connected by a Yellow line
						vector<double> x_waypts; 
						vector<double> y_waypts;

						for (int i=0;i<20;i++)
						{
							x_waypts.push_back(i*4);
							y_waypts.push_back(polyeval(coeffs, x_waypts[i]));
						}

						msgJson["next_x"] = x_waypts; //ptsx_car;
						msgJson["next_y"] = y_waypts; //ptsy_car;


						auto msg = "42[\"steer\"," + msgJson.dump() + "]";
						std::cout << msg << std::endl << std::endl;
						//std::cout << "Steer: " << steer_value << "Throttle: " << throttle_value << endl;

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
						std::cout << "Sent successfully! " << std::endl << std::endl;
					}
				} else 
				{
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
