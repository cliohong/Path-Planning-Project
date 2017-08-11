#ifndef POLYTRAJECTORYGENERATOR_H_
#define POLYTRAJECTORYGENERATOR_H_

#include <vector>
#include <iostream>
#include <map>
#include "struct.h"
#include "vehicle.h"
#include "jmt.h"
#include <math.h>

using namespace std;

class polyTrajectoryGenerator{

  public:
    State target_state_s;
    State target_state_d;
  
    polyTrajectoryGenerator();
  
    PathType update(Vehicle& myCar, vector<Vehicle>& otherCars);
  
    double distanceToClosestCar(const Vehicle &myCar,const vector<Vehicle>& otherCars,const LaneType lane_type, const double factor);
  
//    double distanceToClosestCarBehind(const Vehicle &myCar,const vector<Vehicle>& otherCars,const LaneType lane_type);
//  
    void trajectory(Vehicle& car, const PathType path);
    JMT get_jmt_s() const;
    JMT get_jmt_d() const;
  
  
  private:
    double cur_front_v;
    double cur_front_s;
    vector<JMT>jmt_path;

    double costOfLaneChange(const double front_gap, const double back_gap, const LaneType lane) const;
    double costOfStraightCourse(const double gap);
    double costTotalMinJerk(vector<double>traj_coeffs, const double t);
    double maxJerkCost(vector<double>traj_coeffs,const double t);
    double logistic(double x){return 2/(1+exp(-x))-1 ;}
    vector<double> differentiate(vector<double> coefs);
    double path_jerk_cost(Vehicle&myCar, PathType path);


};

#endif //POLYTRAJECTORYGENERATOR_H_
