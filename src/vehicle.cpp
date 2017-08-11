#include "vehicle.h"

Vehicle::Vehicle(){
  this->lane = LaneType::UNSPECIFIED;
}

void Vehicle::update_position(const double s, const double d){
  this->s = s;
  this->d = d;
  this->lane = this->convert_d_to_lane(this->d);
}

void Vehicle::update_speed(const double v){
  this->v = v;
}

void Vehicle::update_save_states(const State& state_s, const State& state_d){
  this->cur_state_s = state_s;
  this->cur_state_d = state_d;
  this->cur_state_s.pos = fmod(this->cur_state_s.pos, MAX_TRACK_S);
}

void Vehicle::determine_lanes(){

  if (this->lane == LaneType::LEFT) {

    this->lane_at_left = LaneType::NONE;
    this->lane_at_right = LaneType::MID;

  } else if (this->lane == LaneType::MID) {

    this->lane_at_left = LaneType::LEFT;
    this->lane_at_right = LaneType::RIGHT;

  } else if (this->lane == LaneType::RIGHT) {

    this->lane_at_left = LaneType::MID;
    this->lane_at_right = LaneType::NONE;

  } else {

    this->lane = LaneType::UNSPECIFIED;
    this->lane_at_left = LaneType::UNSPECIFIED;
    this->lane_at_right = LaneType::UNSPECIFIED;
  }
}

LaneType Vehicle::convert_d_to_lane(const double d){

  LaneType lane = LaneType::NONE;

  if (d > 0.0 && d < 4.0) {
    lane = LaneType::LEFT;
  } else if (d > 4.0 && d < 8.0) {
    lane = LaneType::MID;
  } else if (d > 8.0 && d < 12.0) {
    lane = LaneType::RIGHT;
  }
  return lane;
}

LaneType Vehicle::convert_d_to_lane(){
  return this->convert_d_to_lane(this->d);
}

double Vehicle::convert_lane_to_d(const LaneType lane){

  double d = 6.;

  if (lane == LaneType::LEFT) {
    d = 2.2;
  } else if (lane == LaneType::MID) {
    d = 6.;
  } else if (lane == LaneType::RIGHT) {
    d = 9.8; //consider running out of lane if speed is too large
  }
  return d;
}

double Vehicle::convert_lane_to_d(){
  return this->convert_lane_to_d(this->lane);
}

double Vehicle::get_target_d(const PathType path){

  if (path == PathType::KEEPLANE) {
    return this->convert_lane_to_d(this->lane);

  } else if (path == PathType::TURNRIGHT) {
    
      return this->convert_lane_to_d(this->lane_at_right);
    
  } else if (path == PathType::TURNLEFT) {
      return this->convert_lane_to_d(this->lane_at_left);
  }

  return this->convert_lane_to_d(this->lane);
}
