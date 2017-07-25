#include "PID.h"

#include <iostream>

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  i_error = d_error = p_error = 0;
  i_error_l2 = 0;
}

PID::PID(int _REQ_CTE_OBSERVATIONS, float ignore_initial) {
  
  REQ_CTE_OBSERVATIONS = _REQ_CTE_OBSERVATIONS;
  IGNORE_FIRST_CTE_OBSERVATIONS = int(REQ_CTE_OBSERVATIONS * ignore_initial);
}


PID::~PID() {}

double PID::get_i_error_l2_with_params(double _Kp, double _Ki, double _Kd) {
  std::cout << "trying with " << _Kp << " " << _Ki << " " << _Kd << std::endl;

  Init(_Kp, _Ki, _Kd);
  double error= get_i_error_l2();
  std::cout << "error: " << error << std::endl;

  return error;
}

double PID::get_i_error_l2() {
  lock.lock();
  double l2_error = i_error_l2;
  lock.unlock();
  return l2_error;
}

void PID::Init(double _Kp, double _Ki, double _Kd) {
  i_error = d_error = p_error = 0;

  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  
  i_error_l2 = 0;
  cte_observations = 0;
  lock.lock();
}

void PID::UpdateError(double cte) {
  i_error += cte;
  d_error  = cte - p_error;
  p_error  = cte;
  
  if (cte_observations < REQ_CTE_OBSERVATIONS) {
    if (cte_observations > IGNORE_FIRST_CTE_OBSERVATIONS) {
      i_error_l2 += cte * cte;
    }
    cte_observations++;
    if (cte_observations == REQ_CTE_OBSERVATIONS) {
      lock.unlock();
    }

  }
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}



