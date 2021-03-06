#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0; 

}

void PID::UpdateError(double cte) 
{
  /**
   * TODO: Update PID errors based on cte.
   */
  double prev_cte = p_error;

  p_error  = cte;
  i_error  = i_error + cte;
  d_error  = cte - prev_cte;
}

double PID::Total_Error_Steer() 
{
   /**
   * TODO: Calculate total error for steering using PID control
   */
  double error_steer = (-Kp*p_error) - (Ki*i_error) - (Kd*d_error);

  return error_steer;
}


double PID::Total_Error_Throttle(double max_throttle)
{
   /**
   * TODO: Calculate total error for throttle using PID control and max throttle limited to
   */
  double error_throttle = max_throttle - Kp*p_error - Ki*i_error - Kd*d_error;

  return error_throttle;
}

