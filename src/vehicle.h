#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <vector>
#include <math.h>
#include "struct.h"

class Vehicle {

  public:

    double s;
    double d;
    double v;
    double front_gap;
    double front_v;
    double front_s;

    State cur_state_s;
    State cur_state_d;

    LaneType lane;
    LaneType lane_at_right;
    LaneType lane_at_left;

    Vehicle();

    void update_position(const double s, const double d);
    void update_speed(const double v);
    void update_save_states(const State& state_s, const State& state_d);
    void determine_lanes();

    LaneType convert_d_to_lane(const double d);
    LaneType convert_d_to_lane();
    double convert_lane_to_d(const LaneType lane);
    double convert_lane_to_d();
    double get_target_d(const PathType path);
};

#endif //VEHICLE_H_
