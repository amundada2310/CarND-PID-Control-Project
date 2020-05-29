#ifndef PID_H
#define PID_H

class PID {
 public:
  
  
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error for steering
   * @output The total PID error for steering
   */
  
   double Total_Error_Steer();

  /**
   * Calculate the total PID error for throttle using max_throttle
   * @output The total PID error for throttle
   */
  double Total_Error_Throttle(double max_throttle);
  
 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

};

#endif  // PID_H