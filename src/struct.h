#ifndef struct_h
#define struct_h

#include <vector>
using namespace std;

// set duration time we sent to sim each time
const double PATH_PLAN_INCREMENT = 0.02;
const double PATH_PLAN_SECONDS = 2.5;
const int NUMBER_OF_POINTS = 125;

//penalty largest
const double MAX_COST = 5000.0;

// how much points left for the controller to perform
// before we start planning again
const int  PATH_SIZE_CUTOFF = 15;

//penalty the car in front/left/right too close
const double LANE_CHANGE_COST_SIDE_F = 1.0;
//penalty the car behind left/right too close
const double LANE_CHANGE_COST_SIDE_R = .8;

//be careful of the track loop !!!
const double MAX_TRACK_S = 6945.554;

// boundaries of acceptable speed of our vehicle
const double HARD_SPEED_LIMIT = 22.352; // 50mph in m/s
const double MAX_SPEED_LIMIT = 20.75;
const double MIN_SPEED_LIMIT = 16.0;

// if the gap is less than this we consider it unsafe to turn
const double FRONT_GAP_THRESH = 25.0;
const double BACK_GAP_THRESH = 15.0;

const double FRONT_BUFFER = 40;
const double DISTANCE_BUFFER = 5.0;
const double SPEED_BUFFER = 4.0;// 6.0;

//tweak the penalty or reward values for making the path smoothier
const double MID_REWARD = 0.35;
const double TURN_PENALTY= 1.5;

const double EXPECTED_JERK_IN_ONE_SEC = 1.0;
//total jerk cost weight
const double TOTAL_JERK_COST_WEIGHT = 10.;
const double MAX_JERK = 0.2;

enum class LaneType {
  LEFT, MID, RIGHT, NONE, UNSPECIFIED
};

enum class PathType {
  KEEPLANE, TURNRIGHT, TURNLEFT
};

//Type storing position, velocity, acceleration components in s & d respectively
struct State {
  double pos;
  double v;
  double acc;
};


//Type storing x, y vectors which are the map coordinates from simulator
struct XYPoints {
  vector<double> xs;
  vector<double> ys;
  int n; //no. of (x,y)pairs
};

#endif /* struct_h */
