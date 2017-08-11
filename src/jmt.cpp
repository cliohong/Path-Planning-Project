#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT(const State& start, const State& end, const double t) {

  MatrixXd A=MatrixXd(3,3);
  VectorXd B=VectorXd(3);
  VectorXd C=VectorXd(3);
  
  double t2=t*t;
  double t3=t*t2;
  double t4=t*t3;
  double t5=t*t4;
  
  A<< t3,t4,t5,
  3*t2, 4*t3,5*t4,
  6*t, 12*t2, 20*t3;
  
  C<<end.pos-(start.pos+start.v*t+0.5*t2*start.acc), end.v-(start.v+start.acc*t), end.acc-start.acc;
  
  B=A.inverse()*C;
  
  this->coeffs = {start.pos,start.v,0.5*start.acc,B[0],B[1],B[2]};
}

double JMT::jmt(double t){
  double t2=t*t;
  double t3=t*t2;
  double t4=t*t3;
  double t5=t*t4;
  double s= this->coeffs[0]+ this->coeffs[1]*t+this->coeffs[2]*t2+ this->coeffs[3]*t3+this->coeffs[4]*t4+this->coeffs[5]*t5;
  return s;
}

double JMT::polyeval(vector<double> coeffs, double t){
    double res =0.0;
    for(int i=0;i<coeffs.size();i++){
      res += coeffs[i]*pow(t,i);
    }
  return res;
}

double JMT::polyderv(vector<double> coeffs, double t){
  double res = 0.0;
  for(int i=1;i<coeffs.size();i++){
    res+=coeffs[i]*i*CppAD::pow(t, i-1);
  }
  return res;
}
//vector<double>JMT::differentiate(vector<double> coefs){
//  vector<double>new_coeffs;
//  for(int i=1;i<coefs.size();i++){
//    new_coeffs.emplace_back((i+1)*coeffs[i]);
//  }
//  return new_coeffs;
//}
