//
// Created by Jeffrey Lutz on 9/12/17.
//

#include <iostream>
#include <cmath>
#include "Car.h"
#include "spline.h"

using namespace std;

Car::Car() {
    ego_lane = 1;
    last_lane_change = 0;
    ref_v = 0.0;
    lane_speed = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
    lane_frontcar_s = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    lane_backcar_s = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};
    ego_state = STRAIGHT;
    setLaneChangeTime();
    cout << "Autonomous Vehicle created at lane 1" << endl;
}

string Car::pad(double d) {
    double dnew = round(d * 10.0) / 10.0;
    string val = " " + to_string(dnew);
    val = val.substr(0,val.length()-5);

    while(val.length() < 6) {
        val = " " + val;
    }
    return val;
}

double Car::pi() { return M_PI; }

double Car::deg2rad(double deg) { return deg * pi() / 180.0; }

double Car::rad2deg(double rad) { return rad * 180.0 / pi(); }

void Car::setLaneChangeTime() {
    time(&last_lane_change);
}

bool Car::isJerkSafe() {
    return getLastLaneChangeDiff() > 2;
}

long Car::getLastLaneChangeDiff() {
    time_t now;
    time(&now);
    return now - last_lane_change;
}

void Car::update_position(double x, double y, double s, double d, double yaw, double speed) {
    ego_x = x;
    ego_y = y;
    ego_s = s;
    ego_d = d;
    ego_yaw = yaw;
    ego_speed = speed;
}

vector<double> Car::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

vector<vector<double>> Car::realize_state(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                          const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                          const vector<double> &map_waypoints_s) {
    /***********************
Based on individual state and the objectives of the state, adjust ref_v so as to achieve the state safely. ego_lane
variable will also be changed once lane change is determined to be safe for execution.
*************************/

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    int prev_size = previous_path_x.size();

    int lane0 = lane_speed[0];
    int lane1 = lane_speed[1];
    int lane2 = lane_speed[2];


    cout << "lane 0: " << lane0 << " lane 1: " << lane1 << " lane 2: " << lane2
         << " -- Selected State: " << ego_state << " Last lane change: " << getLastLaneChangeDiff() << endl;

    if (ego_state == STRAIGHT) {

        //Code to maintain lane speed and sufficient separaton between ego and front car
        if (ref_v < SPEED_LIMIT && lane_frontcar_s[ego_lane] - ego_future_s > FRONT_SAFE_DISTANCE) {

            ref_v += .224;
        }
            //else decrease speed to maintain safety distance and safe speed
        else {

            ref_v -= .224;

            //perform emergency breaking if front car future s and ego_future_s separation is less than 10m
            if (lane_frontcar_s[ego_lane] - ego_future_s < 10) {
                ref_v -= 2.0;
            }

        }

    } else if (ego_state == LEFT) {

        //Code to maintain lane speed and sufficient separaton between ego and front car
        if (ref_v < SPEED_LIMIT && lane_frontcar_s[ego_lane] - ego_future_s > REAR_SAFE_DISTANCE) {

            //if Lane Change Left and velocity is lesser than current lane speed, check target lane speed and reduce speed
            //to 5MPH slower than target lane speed to find opportunity to change lane
            if (ref_v < lane_speed[ego_lane - 1] - 5.0) {

                ref_v += .224;
            } else {
                ref_v -= .224;
            }
        }
            //else decrease speed to maintain safety distance and safe speed
        else {

            ref_v -= .224;

            //perform emergency breaking if front car future s and ego_future_s separation is less than 10m
            if (lane_frontcar_s[ego_lane] - ego_future_s < 10) {
                ref_v -= 2.0;
            }
        }

        //check target lane and accelerate during lane change if safe to change lane
        //maintaince 30m from front car and 20m from back car
        if (lane_frontcar_s[ego_lane - 1] - ego_future_s > REAR_SAFE_DISTANCE) {

            if (ego_future_s - lane_backcar_s[ego_lane - 1] > REAR_SAFE_DISTANCE - 10.0 && isJerkSafe()) {
                setLaneChangeTime();
                ego_lane = ego_lane - 1;
            }
        }

    } else if (ego_state == RIGHT) {

        if (ref_v < SPEED_LIMIT && lane_frontcar_s[ego_lane] - ego_future_s > REAR_SAFE_DISTANCE) {

            if (ref_v < lane_speed[ego_lane + 1] - 5.0) {
                ref_v += .224;
            } else {
                ref_v -= .224;
            }
        }
            //else perform braking as per emergency or normal
        else {

            ref_v -= .224;
            //perform emergency braking if front car future s and ego_future_s separation is less than 10
            if (lane_frontcar_s[ego_lane] - ego_future_s < 10) {
                ref_v -= 2.0;
            }
        }

        //check target lane and accelerate during lane change if safe to change lane
        //maintaince 30m from front car and 20m from back car
        if (lane_frontcar_s[ego_lane + 1] - ego_future_s > REAR_SAFE_DISTANCE) {

            if (ego_future_s - lane_backcar_s[ego_lane + 1] > REAR_SAFE_DISTANCE - 10.0 && isJerkSafe()) {
                setLaneChangeTime();
                ego_lane = ego_lane + 1;
            }
        }
    }

    /***********************
    Trajectory generation using ref_v set in earlier
    *************************/

    //list of spaced (x,y) points evenly spaced at 30m
    vector<double> ptsx;
    vector<double> ptsy;

    //reference x, y and yaw states
    double ref_x = ego_x;
    double ref_y = ego_y;
    double ref_yaw = deg2rad(ego_yaw);

    //if prev path is almost empty, use the car position as starting reference
    if (prev_size < 2) {

        //create previous car position in (x,y) map coordinate using the car current position
        double prev_car_x = ego_x - cos(ego_yaw);
        double prev_car_y = ego_y - sin(ego_yaw);

        //push back into ptsx and ptsy the 2 points for the tangent line
        ptsx.push_back(prev_car_x);
        ptsx.push_back(ego_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ego_y);
    }
        //if prev path is not empty, use previous path endpoint at starting point
    else {

        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        //push back into ptsx and ptsy the 2 points for the tangent line
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    //add another 3 waypoints where each waypoints are spaced 30m apart
    //convert these waypoints from Frenet back to (x,y) map space
    vector<double> next_wp0 = getXY(ego_future_s + 30, (2 + 4 * ego_lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);
    vector<double> next_wp1 = getXY(ego_future_s + 60, (2 + 4 * ego_lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);
    vector<double> next_wp2 = getXY(ego_future_s + 90, (2 + 4 * ego_lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++) {
        //use every previous ptsx and ptsy values (waypoints x,y values)
        //find the relative position of these points to car position
        //and convert them from map space to car space (car reference frame)
        //car is now at (0,0) origin
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        //changing from map space to car space by using rotation matrix for each waypoints
        //https://en.wikipedia.org/wiki/Rotation_matrix
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    }

    //create a spline
    tk::spline s;

    //fit all previous waypoints to the spline
    //waypoints are now in car reference frame
    s.set_points(ptsx, ptsy);

    //push all previous left over untravelled paths into next_x_vals and next_y_vals
    //for continuity in path planning
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    //assume a target_x 30m away
    //fill up additional points within this 30m distance using spline provided y values
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y)); //assume straight line

    double x_add_on = 0;

    //top up the rest of the planner next_x_vals and next_y_vals with new planned points
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

        //find new x_point and y_point
        double N = (target_dist / (.02 * ref_v / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        //update x_add_on to the new point
        x_add_on = x_point;

        //set x_ref and y_ref to new x_point and y_point
        //x_ref and y_ref represents the next point after the car's current location
        double x_ref = x_point;
        double y_ref = y_point;

        //rotate from car space back to map space but without translation
        //i.e. still taking car as centre (0,o)
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        //convert x_point and y_point with translation to account for car's position on map
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);

    }

    return {next_x_vals, next_y_vals};

}

void Car::update_state(const vector<double> &previous_path_x, const double &end_path_s,
                       const vector<vector<double>> &sensor_fusion) {
    int prev_size = previous_path_x.size();

    //variables for cars being checked
    double vx;
    double vy;
    double x;
    double y;
    double check_speed;
    double check_car_s;
    double check_car_future_s;

    //if previous planning was done, set ego_future_s to last known end_path_s point
    if (prev_size > 0) {
        ego_future_s = end_path_s;
    }
        //else set ego_future_s to current ego_s position
    else {
        ego_future_s = ego_s;
    }

    //Reset lane_speed to max road speed limit
    lane_speed = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
    //Reset lane_frontcar_s to max double
    lane_frontcar_s = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    //Reset lane_backcar_s to min double
    lane_backcar_s = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};

    /***********************
    Step 1
    Iterate through all vehicles in all lanes and find vehicle that is closest (in front and behind) to ego
    to determine lane speed

    * Information recorded for all 3 lanes include lane_speed, lane_frontcar_s and lane_backcar_s.
    * Data format for each car is: [id, x, y, vx, vy, s, d].
    *************************/
    for (int i = 0; i < sensor_fusion.size(); i++) {
        double id = sensor_fusion[i][0];
        x = sensor_fusion[i][1];
        y = sensor_fusion[i][2];
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];

        cout << "id/ego_s/ego_d/other_s/other_d/distance: " << pad(id) << " " << pad(ego_s) << " " << pad(ego_d) << " " << pad(s) << " " << pad(d) << " " << pad(s - ego_s) << " " << pad(d - ego_d) << endl;

        check_speed = sqrt(vx * vx + vy * vy);
        check_car_s = sensor_fusion[i][5];
        int lane_index;
        if (d < 4) {
            lane_index = 0;
        } else if (d < 8) {
            lane_index = 1;
        } else if (d < 12) {
            lane_index = 2;
        }
        check_car_future_s = check_car_s + ((double) prev_size * .02 * check_speed);
        double s_diff = abs(check_car_future_s - ego_future_s);
        bool inFront = check_car_future_s > ego_future_s;

        //if vehicle is in front of ego and lesser than 50km/hr at final projected position record
        //target vehicle s location and speed. Only vehicle in front and closest to ego is recorded
        if(inFront) {
            if (s_diff < FRONT_SAFE_DISTANCE &&
                    s_diff < abs(lane_frontcar_s[lane_index] - ego_future_s) ) {
                lane_speed[lane_index] = check_speed * 2.237;
                lane_frontcar_s[lane_index] = check_car_future_s;
            }
        } else {
            //if vehicle is behind ego and lesser than REAR_SAFE_DISTANCE at final projected position record
            //target vehicle s location. Only vehicle behind and closest to ego is recorded.
            if (s_diff < REAR_SAFE_DISTANCE && abs(check_car_future_s - ego_future_s) < abs(lane_backcar_s[lane_index] - ego_future_s)) {
                lane_backcar_s[lane_index] = check_car_future_s;
            }
        }
    }

    /***********************
    Step 2
    Use previously found closest car position and speed in all 3 lanes to determine ego's preferred state
    ego's aim is to complete the circuit safely and in the shortest possible time
    *************************/

    //ideal_lane is the lane that allows highest travel speed
    int ideal_lane = distance(lane_speed.begin(), max_element(lane_speed.begin(), lane_speed.end()));

    // The default is to continue straight in existing lane
    ego_state = STRAIGHT;

    if(ego_lane != ideal_lane) {
        if(ego_lane < ideal_lane) {
            ego_state = RIGHT;
        } else {
            ego_state = LEFT;
        }
    }
}


