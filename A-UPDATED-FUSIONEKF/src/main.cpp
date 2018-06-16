#include <uWS/uWS.h>
#include <math.h>
#include <iostream>
#include "json.hpp"
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "Eigen/Dense"
#include "datapoint.h"
#include "fusionekf.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {

  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");

  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths){

  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for (int k = 0; k < estimations.size(); ++k){

    VectorXd diff = estimations[k] - ground_truths[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}


int main(int argc, char* argv[]) {

  FusionEKF fusionEKF;
  vector<VectorXd> all_estimations;
  vector<VectorXd> all_truths;

  uWS::Hub h;

  h.onMessage([&fusionEKF, &all_truths, &all_estimations](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message
  // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(std::string(data));

      if (s != "") {
        cout << "Got message." << endl;
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          cout << "Got telemetry message." << endl;

          // Data from simulator
          string new_data = j[1]["sensor_measurement"];
          istringstream iss(new_data);
          string sensor_id = "";
          iss >> sensor_id;

          cout << "Got new data (string) from simulator." << endl;
          cout << "Current measurement -- sensor id: " << sensor_id << endl;

          long long timestamp;
          DataPoint sensor_data;

          if (sensor_id.compare("L") == 0) {

            cout << "Getting lidar measurements... " << endl;

            double lidar_px, lidar_py;
            VectorXd lidar_values(2);

            iss >> lidar_px;
            iss >> lidar_py;
            iss >> timestamp;

            cout << "... done. lidar values:" << lidar_px << " and " << lidar_py << "." << endl;
            lidar_values << lidar_px, lidar_py;
            sensor_data.set(timestamp, DataPointType::LIDAR, lidar_values);

          } else if (sensor_id.compare("R") == 0) {

            cout << "Getting radar measurements... " << endl;

            double rho, phi, drho;
            VectorXd radar_values(3);
            iss >> rho;
            iss >> phi;
            iss >> drho;
            iss >> timestamp;

            cout << "...done. radar values:" << rho << "," << phi << " and " << drho << endl;
            radar_values << rho, phi, drho;
            sensor_data.set(timestamp, DataPointType::RADAR, radar_values);
          }

          cout << "Getting ground truth... " << endl;

          double px, py, vx, vy;
          VectorXd truth_values(4);

          iss >> px;
          iss >> py;
          iss >> vx;
          iss >> vy;
          iss >> timestamp;

          truth_values << px, py, vx, vy;
          cout << "...done. ground truth values:"
               << px << "," << py << ","<< vx << " and " << vy << endl;

          all_truths.emplace_back(truth_values);

          fusionEKF.process(sensor_data);
          VectorXd estimate = fusionEKF.get_resulting_state();
          // Contains (px, py, vx, vy) estimate by FusionEKF

          all_estimations.emplace_back(estimate);

          // A measure of how the awesome the Kalman filter is working
          // The smaller, the better
      	  VectorXd RMSE = calculate_RMSE(all_estimations, all_truths);

          cout << "**************************************************" << endl;
          cout << "estimate_px: " << estimate(0) << endl;
          cout << "estimate_py: " << estimate(1) << endl;
          cout << "estimate_vx: " << estimate(2) << endl;
          cout << "estimate_vy: " << estimate(3) << endl;
          cout << "---------------------------------------------------" << endl;
          cout << "truth_values_px: " << truth_values(0) << endl;
          cout << "truth_values_py: " << truth_values(1) << endl;
          cout << "truth_values_vx: " << truth_values(2) << endl;
          cout << "truth_values_vy: " << truth_values(3) << endl;
          cout << "---------------------------------------------------" << endl;
          cout << "rmse_x: " << RMSE(0) << endl;
          cout << "rmse_y: " << RMSE(1) << endl;
          cout << "rmse_vx: " << RMSE(2) << endl;
          cout << "rmse_vy: " << RMSE(3) << endl;
          cout << "**************************************************" << endl;

          json msgJson;

          msgJson["estimate_x"] = estimate(0);
          msgJson["estimate_y"] = estimate(1);

          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);

          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          cout << "...Sent estimate." << endl;
        }

      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
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
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
