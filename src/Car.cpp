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
    cars_speed_front = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
    cars_s_front = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    cars_s_rear = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};
    cars_dist_front = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    cars_dist_rear = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    ego_state = GO_STRAIGHT;
    setLaneChangeTime();
    cout << "Autonomous Vehicle created at lane 1" << endl;
}

string Car::pad(double d, int pad, int trim_right) {
    double dnew = round(d * 10.0) / 10.0;
    string val = " " + to_string(dnew);
    val = val.substr(0, val.length() - trim_right);

    while (val.length() <= pad) {
        val = string(" ").append(val);
    }
    return val;
}

double Car::pi() { return M_PI; }

double Car::deg2rad(double deg) { return deg * pi() / 180.0; }

void Car::setLaneChangeTime() {
    time(&last_lane_change);
}

bool Car::isLaneChangeJerkSafe() {
    return getLastLaneChangeDiff() > WAIT_TIME_BETWEEN_LANE_CHANGES;
}

long Car::getLastLaneChangeDiff() {
    time_t now;
    time(&now);
    return now - last_lane_change;
}

double Car::getSpeedChange(bool increase) {
    if (!increase) {
        return -1.0 * SPEED_CHANGE;
    }
    if (SPEED_CHANGE < SPEED_LIMIT - ego_speed) {
        return SPEED_CHANGE;
    }
    return SPEED_LIMIT - ego_speed;
}

bool Car::hasSafeDistanceToCarsLeftLane() {
    return (ego_lane == 0 ? false : cars_dist_rear[ego_lane - 1] > CAR_SAFE_DIST_REAR)
           &&
           (ego_lane == 0 ? false : cars_dist_front[ego_lane - 1] > CAR_SAFE_DIST_FRONT);
}

bool Car::hasSafeDistanceToCarsRightLane() {
    return (ego_lane == 2 ? false : cars_dist_rear[ego_lane + 1] > CAR_SAFE_DIST_REAR)
           &&
           (ego_lane == 2 ? false : cars_dist_front[ego_lane + 1] > CAR_SAFE_DIST_FRONT);
}

bool Car::attemptLeftLaneChange() {
    if (hasSafeDistanceToCarsLeftLane()) {
        setLaneChangeTime();
        ego_lane = ego_state == GO_LEFT ? ego_lane - 1 : ego_lane + 1;
        cout << "Left lane change made" << endl;
        return true;
    } else {
        return false;
    }
}

bool Car::attemptRightLaneChange() {
    if (hasSafeDistanceToCarsRightLane()) {
        setLaneChangeTime();
        ego_lane = ego_state == GO_LEFT ? ego_lane - 1 : ego_lane + 1;
        cout << "Right lane change made" << endl;
        return true;
    } else {
        return false;
    }
}

bool Car::attemptEitherLaneChange() {
    return attemptLeftLaneChange() || attemptRightLaneChange();
}

vector<double> Car::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    long wp2 = (prev_wp + 1) % maps_x.size();

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

void Car::update_position(double x, double y, double s, double yaw, double speed) {
    ego_x = x;
    ego_y = y;
    ego_s = s;
    ego_yaw = yaw;
    ego_speed = speed;
}

void Car::update_state(const vector<double> &previous_path_x, const double &end_path_s,
                       const vector<vector<double>> &sensor_fusion) {
    long prev_size = previous_path_x.size();

    //if previous planning was done, set ego_future_s to last known end_path_s point
    if (prev_size > 0) {
        ego_future_s = end_path_s;
    }
        //else set ego_future_s to current ego_s position
    else {
        ego_future_s = ego_s;
    }

    //Reset cars_speed_front to max road speed limit
    cars_speed_front = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
    //Reset  to max double
    cars_s_front = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    //Reset cars_s_rear to min double
    cars_s_rear = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};

    // Reset cars_dist_front and cars_dist_rear to max double
    cars_dist_front = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    cars_dist_rear = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};

    /***********************
    Step 1
    Iterate through all vehicles in all lanes and find vehicle that is closest (in front and behind) to ego
    to determine lane speed

    * Information recorded for all 3 lanes include lane_speed, lane frontcar_s and lane_backcar_s.
    * Data format for each car is: [id, x, y, vx, vy, s, d].
    *************************/
    for (const auto &i : sensor_fusion) {
        double vx = i[3];
        double vy = i[4];
        double s = i[5];
        double d = i[6];
        double speed = sqrt(vx * vx + vy * vy);

        int lane_index = 0;
        if (d < 4) {
            lane_index = 0;
        } else if (d < 8) {
            lane_index = 1;
        } else if (d < 12) {
            lane_index = 2;
        }
        double s_future = s + ((double) prev_size * .02 * speed);
        double car_dist_present = abs(cars_s_front[lane_index] - ego_future_s);
        double car_dist_future = abs(s_future - ego_future_s);
        bool car_is_in_front = s_future > ego_future_s;
        double s_diff_rear = abs(cars_s_rear[lane_index] - ego_future_s);

        //if vehicle is in front of ego and lesser than 50km/hr at final projected position record
        //target vehicle s location and speed. Only vehicle in front and closest to ego is recorded
        if (car_is_in_front) {
            double distance_multiplier = (ego_lane == lane_index ? CURRENT_LANE_DISTANCE_MULTIPLIER
                                                                 : OTHER_LANE_DISTANCE_MULTIPLIER);
            if (car_dist_future < (CAR_SAFE_DIST_FRONT * distance_multiplier)
                && car_dist_future < car_dist_present
                    ) {
                cars_speed_front[lane_index] = speed * MPS_TO_MPH;
                cars_s_front[lane_index] = s_future;
                cars_dist_front[lane_index] = car_dist_future;
            }
        } else {
            //if vehicle is behind ego and lesser than CAR_SAFE_DIST_REAR at final projected position record
            //target vehicle s location. Only vehicle behind and closest to ego is recorded.
            if (car_dist_future < CAR_SAFE_DIST_REAR
                && car_dist_future < s_diff_rear
                    ) {
                cars_s_rear[lane_index] = s_future;
                cars_dist_rear[lane_index] = car_dist_future;
            }
        }
    }

    /***********************
    Step 2
    Use previously found closest car position and speed in all 3 lanes to determine ego's preferred state
    *************************/
    //ideal_lane is the lane that allows highest travel speed
    long ideal_lane = distance(cars_speed_front.begin(), max_element(cars_speed_front.begin(), cars_speed_front.end()));

    // The default is to continue straight in existing lane
    ego_state = GO_STRAIGHT;
    bool isEgoLaneSpeedLessThanIdealLaneSpeed = cars_speed_front[ego_lane] < cars_speed_front[ideal_lane];

    double lane_speed_left = ego_lane == 0 ? 0.0 : cars_speed_front[ego_lane - 1];
    double lane_speed_right = ego_lane == 2 ? 0.0 : cars_speed_front[ego_lane + 1];
    if (ego_lane != ideal_lane
        && isEgoLaneSpeedLessThanIdealLaneSpeed
        && isLaneChangeJerkSafe()) {
        if (lane_speed_left == lane_speed_right) {
            ego_state = GO_EITHER;
        } else if (lane_speed_right > lane_speed_left) {
            ego_state = GO_RIGHT;
        } else {
            ego_state = GO_LEFT;
        }
    }
    if (ego_lane != 1 && cars_speed_front[1] == SPEED_LIMIT) {
        // switch to middle lane if not already in middle lane
        // and middle lane is full speed
        // reasoning:  Favor middle lane because if need to switch,
        //             have two other lanes as options.  Improve travel time.
        ego_state = ego_lane == 0 ? GO_RIGHT : GO_LEFT;
    }
}

vector<vector<double>> Car::realize_state(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                          const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                          const vector<double> &map_waypoints_s) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    string ego_lane_str = ego_lane == 0 ? "Left  " : (ego_lane == 1 ? "Middle" : "Right ");
    long prev_size = previous_path_x.size();

    bool hasSafeDistanceToFrontCar = abs(cars_dist_front[ego_lane]) > CAR_SAFE_DIST_FRONT;

    bool hasSafeDistanceToFrontCarLeftLane =
            ego_lane == 0 ? false : cars_dist_front[ego_lane - 1] > CAR_SAFE_DIST_FRONT;
    bool hasSafeDistanceToFrontCarRightLane =
            ego_lane == 2 ? false : cars_dist_front[ego_lane + 1] > CAR_SAFE_DIST_FRONT;
    bool hasSafeDistanceToRearCarLeftLane = ego_lane == 0 ? false : cars_dist_rear[ego_lane - 1] > CAR_SAFE_DIST_REAR;
    bool hasSafeDistanceToRearCarRightLane = ego_lane == 2 ? false : cars_dist_rear[ego_lane + 1] > CAR_SAFE_DIST_REAR;

    bool isEmergencyBrake = cars_dist_front[ego_lane] < EMERGENCY_BRAKE_DISTANCE;
    bool tooCloseToFrontCar = !hasSafeDistanceToFrontCar && ref_v > cars_speed_front[ego_lane];

    string msg = ""
                 //+ "current lane/left/middle/right/steer/last_lane_change: "
                 + ego_lane_str + " -"
                 + pad(cars_dist_front[ego_lane], 3, 7)
                 + pad(cars_speed_front[0], 3, 7)
                 + pad(cars_speed_front[1], 3, 7)
                 + pad(cars_speed_front[2], 3, 7)
                 + " - " + ego_state + " - "
                 + (hasSafeDistanceToRearCarLeftLane && hasSafeDistanceToFrontCarLeftLane ? "L-" : "L*")
                 + " "
                 + (hasSafeDistanceToRearCarRightLane && hasSafeDistanceToFrontCarRightLane ? "R-"
                                                                                            : "R*");
    if (last_msg != msg) {
        last_msg = msg;
        if (banner_counter > 20) {
            cout << "current lane - cur lane dist - left v - middle v - right v - steer - left lane clear - right lane clear - time since last lane change: " << endl;
            banner_counter = 0;
        }
        banner_counter++;
        cout << msg
             << " " << getLastLaneChangeDiff()
             << endl;
    }
    //Make speed control decision independent of steering decision
    if (isEmergencyBrake) {
        ref_v -= SPEED_CHANGE * 2.0;
    } else {
        ref_v += getSpeedChange(!tooCloseToFrontCar);
    }
    if (ref_v > SPEED_LIMIT) {
        ref_v = SPEED_LIMIT;
    }
    // Make steering decision independent to speed control decision
    if (ego_state == GO_EITHER) {
        attemptEitherLaneChange();
    } else if (ego_state == GO_LEFT) {
        attemptLeftLaneChange();
    } else if (ego_state == GO_RIGHT) {
        attemptRightLaneChange();
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
