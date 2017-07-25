#ifndef PID_H
#define PID_H

#include <mutex>
#include <condition_variable>

class PID {
private:
  std::mutex * lock;
  std::condition_variable cv;
  
  int REQ_CTE_OBSERVATIONS;
  int cte_observations;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  double i_error_l2;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
  * Constructor
  */
  PID();
  PID(int _REQ_CTE_OBSERVATIONS, std::mutex & _lock);


  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  double get_i_error_l2() const;
  
  double get_i_error_l2_with_params(double, double, double) ;

};

#endif /* PID_H */
