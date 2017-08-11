#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/Eigen-3.3/Eigen/Dense"
#include "struct.h"
#include <cppad/cppad.hpp>
#include<math.h>

class JMT {

  public:
    vector<double>coeffs;
    JMT(const State&start,const State&end, double t);
    double jmt(double t);
  
    double polyeval(vector<double> coeffs, double t);
    double polyderv(vector<double> coeffs, double t);
//    vector<double> differentiate(vector<double> coefs);
  
};

#endif // JMT_H_
