//
// Created by Jeffrey Lutz on 9/12/17.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <time.h>

using namespace std;

class Car {

private:
    const int WAIT_TIME_BETWEEN_LANE_CHANGES = 3; // seconds between lane change to minimize jerk
    const double CURRENT_LANE_DISTANCE_MULTIPLIER = 1.10;
    const double MPS_TO_MPH = 2.237;  // m/s to mph factor
    const double OTHER_LANE_DISTANCE_MULTIPLIER = 1.5;
    const double EMERGENCY_BRAKE_DISTANCE = 10.0;
    const double CAR_SAFE_DIST_FRONT = 30.0; // meters
    const double CAR_SAFE_DIST_REAR = 20.0; //meters
    const double SPEED_LIMIT = 49.95;// 49.5mph = 22.098m/s
    const double SPEED_CHANGE = 0.25;
    const string GO_STRAIGHT = "^";
    const string GO_LEFT = "<";
    const string GO_RIGHT = ">";
    const string GO_EITHER = "-";

    int ego_lane;
    time_t last_lane_change;
    double ego_x;
    double ego_y;
    double ego_s;
    double ego_yaw;
    double ego_speed;
    double ref_v;
    double ego_future_s;
    string ego_state;
    vector<double> cars_speed_front;
    vector<double> cars_s_front;
    vector<double> cars_s_rear;
    vector<double> cars_dist_front;
    vector<double> cars_dist_rear;
    string last_msg;

    double pi();
    double deg2rad(double deg);
    double rad2deg(double rad);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
    void setLaneChangeTime();
    long getLastLaneChangeDiff();
    bool isLaneChangeJerkSafe();
    double getSpeedChange(bool increase);
    bool hasSafeDistanceToCarsLeftLane();
    bool hasSafeDistanceToCarsRightLane();
    bool attemptLeftLaneChange();
    bool attemptRightLaneChange();
    bool attemptEitherLaneChange();

    string pad(double d);
    string pad(double d,int pad, int trim_right);

public:
    Car();

    void update_position(double x, double y, double s, double d, double yaw, double speed);

    void update_state(const vector<double> &previous_path_x, const double &end_path_s,
                      const vector <vector<double>> &sensor_fusion);

    vector <vector<double>> realize_state(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                          const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                          const vector<double> &map_waypoints_s);
};


#endif //PATH_PLANNING_CAR_H
