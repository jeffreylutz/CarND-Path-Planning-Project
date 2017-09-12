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
    int ego_lane;
    time_t last_lane_change;
    double MAX_SPEED = 80.0;
    double ego_x;
    double ego_y;
    double ego_s;
    double ego_d;
    double ego_yaw;
    double ego_speed;
    double FRONT_SAFE_DISTANCE = 50.0; // meters
    double REAR_SAFE_DISTANCE = 50.0; //meters
    double ref_v;
    double SPEED_LIMIT = 49.75;// 49.5mph = 22.098m/s
    double ego_future_s;
    string STRAIGHT = "^";
    string LEFT = "<";
    string RIGHT = ">";
    string ego_state; // state includes KL - Keep Lane / LCL - Lane Change Left / LCR - Lane Change Right
    vector<double> lane_speed;
    vector<double> lane_frontcar_s;
    vector<double> lane_backcar_s;

    double pi();
    double deg2rad(double deg);
    double rad2deg(double rad);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
    void setLaneChangeTime();
    long getLastLaneChangeDiff();
    bool isJerkSafe();
//    bool isFrontCarFasterThanMe(double )

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
