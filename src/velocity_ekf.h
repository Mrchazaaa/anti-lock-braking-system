#ifndef ABS_LIBRARY_VELOCITY_EKF_H_
#define ABS_LIBRARY_VELOCITY_EKF_H_

#include "tinyekf_config.h"

struct VelocityEkf {
  double state[EKF_N];
  double covariance[EKF_N * EKF_N];
  double process_noise[EKF_N * EKF_N];
  double measurement_noise[EKF_M * EKF_M];
  double input[EKF_M];
};

void velocity_ekf_reset(VelocityEkf *ekf, double initial_velocity);
double velocity_ekf_step(
    VelocityEkf *ekf,
    double driving_angle,
    double front_wheel_vel,
    double rear_wheel_vel);

#endif  // ABS_LIBRARY_VELOCITY_EKF_H_
