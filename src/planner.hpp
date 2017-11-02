#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>
#include <math.h>
#include "spline.h"


using namespace std;

class Planner{
	
private:
	const int LEFT_LANE = 0;
	const int MIDDLE_LANE = 1;
	const int RIGHT_LANE = 2;
	
	
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	
		
	double deg2rad(double x) { return x * M_PI / 180; }
	
	
	// Transform from Frenet s,d coordinates to Cartesian x,y
	/////////////////////////////////////////////////////////////////////////
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
	{
		int prev_wp = -1;

		while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
		{
			prev_wp++;
		}

		int wp2 = (prev_wp+1)%maps_x.size();

		double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
		// the x,y,s along the segment
		double seg_s = (s-maps_s[prev_wp]);

		double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
		double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

		double perp_heading = heading-M_PI/2;

		double x = seg_x + d*cos(perp_heading);
		double y = seg_y + d*sin(perp_heading);

		return {x,y};

	}



public:
	tk::spline trajectory;
		
	double ref_x;
    double ref_y;
	double ref_yaw;;
	
	

	Planner(){}
	
	void setMapWaypoints(vector<double>& x, vector<double>& y, vector<double>& s){
		map_waypoints_x = x;
		map_waypoints_y = y;
		map_waypoints_s = s;
	}
	
	void behavioralUpdate(double &car_s, int &current_lane, double &velocity,  vector<vector<double>> sensor_fusion, double &end_path_s, int &path_size){
   
		double car_s_predicted;

		if (  path_size > 0){
			car_s_predicted = end_path_s;
		}
		else{
			car_s_predicted = car_s;
		}

		vector<bool> too_close(3, false);
		bool warning = false;

		int vehicle_lane = LEFT_LANE;
		for(int i = 0; i < sensor_fusion.size(); ++i){
			float d = sensor_fusion[i][6];
			if (d <=4 && d >=0)
				vehicle_lane = LEFT_LANE;
			else if (d >4 && d <=8)
				vehicle_lane = MIDDLE_LANE;
			else if (d >8)
				vehicle_lane = RIGHT_LANE;


			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double v_speed = sqrt(vx*vx + vy*vy);
			double v_s = sensor_fusion[i][5];

			//considering its speed
			double v_s_predicted = v_s + path_size * 0.02 * v_speed;

			// if car is ahead of our car and its closer than 30 meter
			if ( abs(v_s - car_s) < 10 || abs(v_s_predicted - car_s_predicted) < 5 || (abs(v_s_predicted - car_s) < 10) || (abs(v_s - car_s_predicted) < 5)){          
				too_close[vehicle_lane] = true;
			}
			
			if (d < (current_lane * 4 + 4) && d > (4 * current_lane)){
				if ( v_s_predicted > car_s_predicted && (v_s_predicted - car_s_predicted) < 30){				
					warning = true;
				}
			}              
		}
					cout << too_close[0] << "  " << too_close[1] << "  " << too_close[2]  << endl;

        // Decide to change lane 
		if (warning){              
			if (current_lane == LEFT_LANE){
				if (too_close[MIDDLE_LANE] == false){
				  if(velocity < 49.5)
				      velocity += 0.448;	
				  current_lane = MIDDLE_LANE;
				}
				else{
			  		velocity -= 0.224;
				}
			}
			
			else if (current_lane == MIDDLE_LANE){
				if (too_close[LEFT_LANE] == false){
				  if(velocity < 49.5)
				      velocity += 0.448;
				  current_lane = LEFT_LANE;
				}
				else if (too_close[RIGHT_LANE] == false){
					if(velocity < 49.5)
				      velocity += 0.448;
			  		current_lane = RIGHT_LANE;
				}else{
		  			velocity -= 0.224;
				}
			}
			
			else if (current_lane == RIGHT_LANE){
				if (too_close[MIDDLE_LANE] ==false){
					if(velocity < 49.5)
				      velocity += 0.448;
		  			current_lane = MIDDLE_LANE;
				}
				else{
		  			velocity -= 0.224;
				}
			}
		}
		else if(velocity < 49.5){
			velocity += 0.448;
		}

		too_close.clear();
	}
	
	
	void motionPlannerUpdate(double &car_x, double &car_y, double &car_s, double &car_yaw, int &current_lane,  vector<double> previous_path_x, vector<double> previous_path_y){
				
		vector<double> pts_x;
		vector<double> pts_y;
		
		ref_x = car_x;
		ref_y = car_y;
		ref_yaw = deg2rad(car_yaw);
		
		          
		int path_size = previous_path_x.size();
			
		if(path_size < 2)
		{
			double pre_car_x = car_x - cos(car_yaw);
			double pre_car_y = car_y - sin(car_yaw);
			pts_x.push_back(pre_car_x);
			pts_x.push_back(car_x);
			pts_y.push_back(pre_car_y);
			pts_y.push_back(car_y);                                
		}
		else
		{
			ref_x = previous_path_x[path_size-1];
			ref_y = previous_path_y[path_size-1];
			double ref_x_prev = previous_path_x[path_size-2];
			double ref_y_prev = previous_path_y[path_size-2];
			pts_x.push_back(ref_x_prev);
			pts_x.push_back(ref_x);
			pts_y.push_back(ref_y_prev);
			pts_y.push_back(ref_y);
			ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
		}
          
		// Adding three more points far away to ensure a smooth trajectory		
		vector<double> next_wp0 = getXY(car_s + 60, (current_lane * 4 + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s + 90, (current_lane * 4 + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s + 110, (current_lane * 4 + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
				
		pts_x.push_back(next_wp0[0]);
		pts_x.push_back(next_wp1[0]);
		pts_x.push_back(next_wp2[0]);

		pts_y.push_back(next_wp0[1]);
		pts_y.push_back(next_wp1[1]);
		pts_y.push_back(next_wp2[1]);
          
            
            
		// Shift car to (0,0) with ang(abs(v_s_predicted - car_s) < 40)le 0
		for(int i = 0; i < pts_x.size(); i++)
		{                    
		  double shift_x = pts_x[i] - ref_x;
		  double shift_y = pts_y[i] - ref_y;

		  pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		  pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));                                
		}

		trajectory.set_points(pts_x, pts_y);
		
	}


};
#endif