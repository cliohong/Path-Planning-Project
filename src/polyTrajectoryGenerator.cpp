#include "polyTrajectoryGenerator.h"

using namespace std;

polyTrajectoryGenerator::polyTrajectoryGenerator() {}

PathType polyTrajectoryGenerator::update(Vehicle& myCar, vector<Vehicle>& otherCars) {

  myCar.front_gap = this->distanceToClosestCar(myCar, otherCars, myCar.lane,1.);
  myCar.front_v = this->cur_front_v;
  myCar.front_s = this->cur_front_s;
  
  const double straight_cost = this->costOfStraightCourse(myCar.front_gap);
  //if turn right
  const double frontright = this->distanceToClosestCar(myCar, otherCars, myCar.lane_at_right,1.);
  const double backright = this->distanceToClosestCar(myCar, otherCars, myCar.lane_at_right,-1.);
  
  //if turn left
  const double frontleft = this->distanceToClosestCar(myCar, otherCars, myCar.lane_at_left,1.);
  const double backleft = this->distanceToClosestCar(myCar, otherCars, myCar.lane_at_left,-1.);
  
  cout << "|" << endl;
  cout << "| LOOK LEFT..." << endl;
  const double left_cost = this->costOfLaneChange(frontleft, backleft, myCar.lane_at_left);
  
  double l_jerk_cost=path_jerk_cost(myCar, PathType::TURNLEFT);
  double total_l_cost = (left_cost + l_jerk_cost)*1.4;
  
  
  
  cout << "|" << endl;
  cout << "| LOOK RIGHT..." << endl;
  const double right_cost = costOfLaneChange(frontright, backright, myCar.lane_at_right);
  
  double r_jerk_cost=path_jerk_cost(myCar, PathType::TURNRIGHT);
  double total_r_cost = (right_cost + r_jerk_cost)*1.4;
  

  
  double k_jerk_cost=path_jerk_cost(myCar, PathType::KEEPLANE);
  double total_staright_cost=straight_cost*1.2 ;;//+ k_jerk_cost*0.65;
  
  cout <<"---------------------------------------"<<endl;
  
  cout<<"| "<<"turn_left_cost"<<"  | "<<"keep_lane cost"<<"  | "<<"turn_right_cost"<<endl;
  
  cout << "|  " << total_l_cost << " | " <<total_staright_cost << " | " << total_r_cost<<endl;
  cout<<endl;
  
  
  map<double,PathType>cost_map={{total_l_cost,PathType::TURNLEFT},{total_staright_cost,PathType::KEEPLANE},{total_r_cost,PathType::TURNRIGHT}};
  map<double, PathType>::iterator cost_map_iter;
  cost_map_iter=cost_map.begin();
  PathType path=cost_map_iter->second;
  
  if(path == PathType::TURNLEFT){
    cout<<" DECISION: TURN LEFT... "<<endl;
    cout<<"<<<--"<<endl;
    cout<<"     |"<<endl;
    cout<<"     |"<<endl;
    cout<<endl;
  }else if(path == PathType::KEEPLANE){
    cout<<" DECISION: Go Straight... "<<endl;
    cout<<"   | "<<endl;
    cout<<"   | "<<endl;
    cout<<"   | "<<endl;
    cout<<endl;
  }else if (path == PathType::TURNRIGHT){
    cout<<" DECISION: TURN RIGHT... "<<endl;
    cout<<"--->>>"<<endl;
    cout<<"|"<<endl;
    cout<<"|"<<endl;
    cout<<endl;
  }

  return path;
}

double polyTrajectoryGenerator::costOfLaneChange(const double front_gap, const double back_gap, const LaneType lane) const {

  double cost = LANE_CHANGE_COST_SIDE_F / front_gap + LANE_CHANGE_COST_SIDE_R / back_gap; //don't need to care about cars behind us

  if (lane == LaneType::NONE || lane == LaneType::UNSPECIFIED) {
    cout << " No lane to go .... \n" << endl;
    return MAX_COST;
  }

  if (front_gap < FRONT_GAP_THRESH || back_gap < BACK_GAP_THRESH) {

    cout << "Warning! Tooooooo close!!!" << endl;
    cout <<"---------------------------------------"<<endl;
    cout << " GAP - front: " << front_gap << " back: " << back_gap << endl;

    return MAX_COST;
  }

  return cost;
}


double polyTrajectoryGenerator::distanceToClosestCar(
  const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type, const double factor) {

  if (lane_type == LaneType::NONE || lane_type == LaneType::UNSPECIFIED) {
    return 0.0001;
  }

  double closest = MAX_COST;

  for(auto &otherCar: otherCars){

    double front_dist =(otherCar.s - myCar.s)*factor;

    if (otherCar.lane == lane_type && front_dist > 0.0 && front_dist < closest) {
      closest = front_dist;
      this->cur_front_v = otherCar.v;
      this->cur_front_s = otherCar.s;
    }
  }

  return closest;
}

//double polyTrajectoryGenerator::distanceToClosestCarBehind(const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type) {
//  
//  if (lane_type == LaneType::NONE || lane_type == LaneType::UNSPECIFIED) {
//    return 0.0001;
//  }
//  
//  double closest = MAX_COST;
//  
//  for(auto &otherCar: otherCars){
//    
//    double behind_dist = myCar.s - otherCar.s;
//    
//    if (otherCar.lane == lane_type && behind_dist > 0.0 && behind_dist < closest) {
//      closest = behind_dist;
//      this->cur_front_v = otherCar.v;
//      this->cur_front_s = otherCar.s;
//    }
//  }
//  
//  return closest;
//}

double polyTrajectoryGenerator::costOfStraightCourse(const double gap){
  if(gap < FRONT_GAP_THRESH){
    return MAX_COST;
  }
  
  return LANE_CHANGE_COST_SIDE_F/gap;
}



void polyTrajectoryGenerator::trajectory(Vehicle& car, const PathType path){
  
  //get target states
  double target_s = car.cur_state_s.pos + PATH_PLAN_SECONDS*car.cur_state_s.v;
  double target_v = car.cur_state_s.v;
  if(path == PathType::KEEPLANE){
    //increase our car's speed if we are too far from other cars or the car front us is too fast
    if(car.front_v > MAX_SPEED_LIMIT ||car.front_gap > FRONT_BUFFER){
      target_v=MAX_SPEED_LIMIT;
    }else{
      target_v = car.front_v - SPEED_BUFFER;
    }
    if(target_v<MIN_SPEED_LIMIT){
      target_v=MIN_SPEED_LIMIT;
    }
    target_s = car.cur_state_s.pos + PATH_PLAN_SECONDS * 0.5*(target_v + car.cur_state_s.v);
  }
  
  this->target_state_s = {target_s,target_v,0.0}; //assume acc is 0
  this->target_state_d = {car.get_target_d(path),0.0,0.0};
  
  //generate JMT for s,d respectively
  JMT jmt_s(car.cur_state_s, this->target_state_s,PATH_PLAN_SECONDS);
  JMT jmt_d(car.cur_state_d, this->target_state_d,PATH_PLAN_SECONDS);
  this->jmt_path.emplace_back(jmt_s);
  this->jmt_path.emplace_back(jmt_d);
}

JMT polyTrajectoryGenerator::get_jmt_s()const{
  return this->jmt_path[0];
}
JMT polyTrajectoryGenerator::get_jmt_d()const{
  return this->jmt_path[1];
}

vector<double>polyTrajectoryGenerator::differentiate(vector<double> coefs){
  vector<double>new_coeffs;
  for(int i=1;i<coefs.size();i++){
    new_coeffs.emplace_back((i+1)*coefs[i]);
  }
  return new_coeffs;
}

double polyTrajectoryGenerator::costTotalMinJerk(vector<double>traj_coeffs, const double t){
  vector<double>s_dot  = differentiate(traj_coeffs);
  vector<double>s_ddot = differentiate(s_dot);
  vector<double>jerk   = differentiate(s_ddot);
  double total_jerk =0.0;
  for(double delta_t =PATH_PLAN_INCREMENT;delta_t<t+0.001;delta_t+=PATH_PLAN_INCREMENT){
    for(int i=0;i<jerk.size();i++){
      total_jerk+=abs(jerk[i]*pow(delta_t, i));
    }
//    total_jerk*=PATH_PLAN_INCREMENT;
  }
//  double jerk_per_sec=total_jerk/t;
//  return logistic(jerk_per_sec/EXPECTED_JERK_IN_ONE_SEC);
  return logistic(total_jerk);
}

double polyTrajectoryGenerator::maxJerkCost(vector<double>traj_coeffs,const double t){
  vector<double>s_dot  = differentiate(traj_coeffs);
  vector<double>s_ddot = differentiate(s_dot);
  vector<double>jerk   = differentiate(s_ddot);
  double max_jerk = -999999;
  for(double j = PATH_PLAN_INCREMENT;j<t+0.0001;j+=PATH_PLAN_INCREMENT){
    for(int i=0;i<jerk.size();i++){
      double abs_jerk = abs(jerk[i]*pow(j,i));
      if(abs_jerk > max_jerk){
        max_jerk=abs_jerk;
      }
    }
  }
  if(max_jerk>MAX_JERK){
    return 100.;
  }else{
    return 0.0;
  }
}



double polyTrajectoryGenerator::path_jerk_cost(Vehicle&myCar,PathType path){
  double total_jerk_cost = 0.0;
  double target_s = myCar.cur_state_s.pos + PATH_PLAN_SECONDS*myCar.cur_state_s.v;
  double target_v = myCar.cur_state_s.v;
  
  if(path == PathType::TURNLEFT){
    State l_target_state = {target_s,target_v,0};
    JMT jmt_l(myCar.cur_state_s,l_target_state,PATH_PLAN_SECONDS);
    vector<double>l_jmt = jmt_l.coeffs;
    total_jerk_cost=costTotalMinJerk(l_jmt, PATH_PLAN_SECONDS);
    total_jerk_cost+=maxJerkCost(l_jmt, PATH_PLAN_SECONDS);
    return total_jerk_cost*TOTAL_JERK_COST_WEIGHT;
    
  }else if(path ==PathType::TURNRIGHT){
    State r_target_state = {target_s,target_v,0};
    JMT jmt_r(myCar.cur_state_s,r_target_state,PATH_PLAN_SECONDS);
    vector<double>r_jmt = jmt_r.coeffs;
    total_jerk_cost=costTotalMinJerk(r_jmt, PATH_PLAN_SECONDS);
    total_jerk_cost+=maxJerkCost(r_jmt, PATH_PLAN_SECONDS);
    return total_jerk_cost*TOTAL_JERK_COST_WEIGHT;
  }
//  }else if(path ==PathType::KEEPLANE){
//    if(myCar.front_v > MAX_SPEED_LIMIT ||myCar.front_gap > FRONT_BUFFER){
//          target_v=MAX_SPEED_LIMIT;
//      }else{
//        target_v = myCar.front_v - SPEED_BUFFER;
//      }
//    if(target_v<MIN_SPEED_LIMIT){
//          target_v=MIN_SPEED_LIMIT;
//    }
//       
//    double target_ss = myCar.cur_state_s.pos + PATH_PLAN_SECONDS * 0.5*(target_v + myCar.cur_state_s.v);
//    State k_target_state = {target_ss,target_v,0.0};
//    JMT jmt_k(myCar.cur_state_s,k_target_state,PATH_PLAN_SECONDS);
//    vector<double>k_jmt = jmt_k.coeffs;
////    total_jerk_cost=costTotalMinJerk(k_jmt, PATH_PLAN_SECONDS);
////    total_jerk_cost=maxJerkCost(k_jmt, PATH_PLAN_SECONDS);
//    return total_jerk_cost*TOTAL_JERK_COST_WEIGHT;
//  }
  return total_jerk_cost*TOTAL_JERK_COST_WEIGHT;
}
