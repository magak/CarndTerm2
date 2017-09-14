#ifndef PID_H
#define PID_H


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * twiddling is done
  */
  bool twiddleDone_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Returns control value
   */
  double GetControlValue();

  /*
   * Make twiddle iteration
   */
  bool Twiddle(double tol);

private:

  /*
   * Whether the first cte received
   */
  bool firstCteReceived_;

  /*
   * value for control
   */
  double controlValue_;

  /*
   * PID coefficient index for twiddling
   */
  int twiddleIndex_;

  enum class TwiddleState { NotInitialized, BeforeDpIncrease, AfterDpIncrease, AfterDpDecrease };

  /*
   * twiddling state
   */
  TwiddleState twiddleState_;

  /*
   * iteration number
   */
  int n_;

  /*
   * Coefficients for twiddling
   */
  double p_[3];

  /*
   * Deltas for twiddling
   */
  double dp_[3];

  /*
   * Total cte
   */
  double totalCte_;

  /*
   * Best erorr;
   */
  double bestError_;

  /*
   * reset pid
   */
  void reset();

  /*
   * return dp sum
   */
  double getDpSum();

};

#endif /* PID_H */
