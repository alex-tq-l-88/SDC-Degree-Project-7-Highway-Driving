#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

//Helper function to evaluate whether cars in target lane is too close to ego vehicle for a lane change
bool EvaluateDistanceOfCars(json sensor_fusion, int lane, double car_s, int prev_size, bool look_behind){
	for(int i=0;i<sensor_fusion.size();i++){ //check each data point from sensor fusion
      float d= sensor_fusion[i][6]; //Get d coordinate
      if(  d > (2+lane*4 -2) && d<(2+ 4*lane + 2)){ //check lanees
		double vx= sensor_fusion[i][3];
		double vy= sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx +vy*vy);
		double check_car_s = sensor_fusion[i][5];

		check_car_s += ((double)prev_size*0.02*check_speed); 
		
		if(((check_car_s > car_s) && ((check_car_s - car_s) < 30)) || (look_behind && (check_car_s < car_s) && ((car_s - check_car_s <10)))){ //Check other cars' distance ahead of us. If front car is less than 30 and back car more than 1mm
			return true; //true if conditions are met
        }
      }
    }
  return false; //false if conditions are met
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
int lane = 1;
double ref_vel = 0;

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
			vector<double> next_x_vals;
          	vector<double> next_y_vals;
			int prev_size = previous_path_x.size();
          
          /*******NEW CODE STARTS HERE************/
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          if(prev_size > 2){	
            car_s = end_path_s;
          }
          
          bool cars_are_too_close = false; //default setting
          cars_are_too_close = EvaluateDistanceOfCars(sensor_fusion, lane, car_s, prev_size, false); //check if cars are too close
          if(cars_are_too_close){ //check which lane car is too close
            cout<<"Other cars are too close to ego"<<endl;
            //define left and right lane numbers
            int right_lane = lane + 1;
            int left_lane = lane - 1;
            if(left_lane >=0){
              bool cars_are_too_close_left = EvaluateDistanceOfCars(sensor_fusion, left_lane, car_s, prev_size,true);
              if(cars_are_too_close_left){
                cout<<"The left car is too close"<<left_lane<<endl;
                if(right_lane <= 2){ //Check at the eedge
                  bool cars_are_too_close_right = EvaluateDistanceOfCars(sensor_fusion, right_lane, car_s, prev_size, true);
                  if(cars_are_too_close_right){
					//Stay in current lane
                  } else {
					lane = right_lane; //Update target lane
                  }
                } else {
                  bool cars_are_too_close_left_1 = EvaluateDistanceOfCars(sensor_fusion, left_lane, car_s, prev_size,true);
                  if(cars_are_too_close_left_1){
					// Stay in current lane
                  } else {
					lane= left_lane; //update target lane
                  }
                }
              } else {
                lane= left_lane; //update targeete lanee
              }
            } else {
              cout<<"Try right lane as left lane is unavailable"<<endl;
              bool cars_are_too_close_right = EvaluateDistanceOfCars(sensor_fusion, right_lane, car_s, prev_size, true);
              if(cars_are_too_close_right) {
					//Stay in current lane
              } else {
                lane = right_lane; //update lane
              }
            }
            ref_vel -= .224; //slow down
          } else if(ref_vel <49.5) {
            ref_vel += .224; //speed up if too slow
          }
          
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);


		// Get 5 points for spline (1 = previous point, 2 =  current point, 3-5 = previous)
		if(prev_size < 2){ //at start of journey
          double prev_car_x = car_x - cos(car_yaw);
          double prev_car_y = car_y - sin(car_yaw);
          
          ptsx.push_back(prev_car_x); //use last point 
          ptsy.push_back(prev_car_y);
          ptsx.push_back(car_x);//use current point
          ptsy.push_back(car_y);
        } else {
          // take points from previous points 
          double ref_x_prev = previous_path_x[prev_size-2];
          double ref_y_prev = previous_path_y[prev_size-2];
          
          ptsx.push_back(ref_x_prev);
          ptsy.push_back(ref_y_prev);
          
          ref_x = previous_path_x[prev_size-1];
          ref_y = previous_path_y[prev_size -1];
          
          ptsx.push_back(ref_x);
          ptsy.push_back(ref_y);
          
          ref_yaw = atan2(ref_y - ref_y_prev ,  ref_x- ref_x_prev);
        }
          
          //Find remaining 3 points
          vector<double> XY_3rd_point= getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> XY_4th_point= getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> XY_5th_point= getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //3rd point
          ptsx.push_back(XY_3rd_point[0]);
          ptsy.push_back(XY_3rd_point[1]);
          //4th point
          ptsx.push_back(XY_4th_point[0]);
          ptsy.push_back(XY_4th_point[1]);
          //5th point
          ptsx.push_back(XY_5th_point[0]);
          ptsy.push_back(XY_5th_point[1]);
          
          //Transform from global to vehicle coordinates
          for(int i=0; i < ptsx.size(); i++){
            double shift_x_coordinate = ptsx[i] -  ref_x;
            double shift_y_coordinate = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x_coordinate * cos(-ref_yaw) - shift_y_coordinate*sin(-ref_yaw);
            ptsy[i] = shift_x_coordinate * sin(-ref_yaw) + shift_y_coordinate*cos(-ref_yaw);
          }
          
          //create spline for vehicle path
          tk:: spline s;
          //pass all 5 points to spline
          s.set_points(ptsx,ptsy);
          
          //add previous points (second set of points with path waypoints)
          for(int i=0;i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
						//break up points to travel at target speed
						double target_x_coordinate = 30.0;
						double target_y_coordinate = s(target_x_coordinate);
						double target_distance =  sqrt((target_x_coordinate * target_x_coordinate) + (target_y_coordinate * target_y_coordinate) );

						double x_add_on = 0;
						for(int i=1; i<= 50-previous_path_x.size(); i++){
							double N = target_distance / (0.02 * ref_vel / 2.24) ;
							double x_point = x_add_on +  target_x_coordinate /N;
							double y_point = s(x_point);
							x_add_on=x_point;
							double x_ref = x_point;
							double y_ref = y_point;

							x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
							y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
							x_point += ref_x;
							y_point += ref_y;
							next_x_vals.push_back(x_point);
							next_y_vals.push_back(y_point);
						}
          
          /**********New code ends here********/
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
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