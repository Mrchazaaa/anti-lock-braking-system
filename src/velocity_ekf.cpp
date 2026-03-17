#include "velocity_ekf.h"

#include <cmath>
#include <cstring>

#define _float_t double
extern "C" {
#include "tinyekf.h"
}

namespace {

struct VehicleModelParams {
  double mass;
  double wheel_radius;
  double gravity;
  double tire_c1;
  double tire_c2;
  double tire_c3;
};

constexpr VehicleModelParams kVehicleModel = {
    1265.0,
    0.3179,
    9.81,
    1.28,
    23.99,
    0.52,
};
constexpr double kDiffStep = 1e-5;
constexpr double kMinMagnitude = 1e-6;
constexpr double kSlipForceThreshold = 0.001;

static double clamp_nonzero(double value)
{
  return std::fabs(value) < kMinMagnitude
             ? (value < 0.0 ? -kMinMagnitude : kMinMagnitude)
             : value;
}

static void initialize_noise(VelocityEkf *ekf)
{
  std::memset(ekf->process_noise, 0, sizeof(ekf->process_noise));
  std::memset(ekf->measurement_noise, 0, sizeof(ekf->measurement_noise));
  ekf->process_noise[0] = 0.1;
  ekf->process_noise[4] = 0.1;
  ekf->process_noise[8] = 0.1;
  ekf->measurement_noise[0] = 0.00001;
  ekf->measurement_noise[4] = 0.01;
  ekf->measurement_noise[8] = 0.01;
}

static ekf_t make_tiny_ekf(const VelocityEkf &ekf_state)
{
  ekf_t ekf = {};
  std::memcpy(ekf.x, ekf_state.state, sizeof(ekf.x));
  std::memcpy(ekf.P, ekf_state.covariance, sizeof(ekf.P));
  return ekf;
}

static void store_tiny_ekf(const ekf_t &tiny_ekf, VelocityEkf *ekf_state)
{
  std::memcpy(ekf_state->state, tiny_ekf.x, sizeof(tiny_ekf.x));
  std::memcpy(ekf_state->covariance, tiny_ekf.P, sizeof(tiny_ekf.P));
}

static void state_transition(
    const double state[EKF_N],
    const double input[EKF_M],
    double out[EKF_N])
{
  const double x_velocity = clamp_nonzero(state[0]);
  const double y_velocity = state[1];
  const double surface_scale = state[2];

  const double driving_angle = input[0];
  const double front_wheel_velocity = input[1];
  const double rear_wheel_velocity = input[2];

  const double front_slip_angle = driving_angle - std::atan(y_velocity / x_velocity);
  const double rear_slip_angle = -std::atan(y_velocity / x_velocity);

  const double vehicle_speed = std::sqrt(y_velocity * y_velocity + x_velocity * x_velocity);
  const double safe_vehicle_speed = clamp_nonzero(vehicle_speed);

  const double front_longitudinal_slip =
      -((kVehicleModel.wheel_radius * front_wheel_velocity * std::cos(front_slip_angle)) -
        vehicle_speed) /
      safe_vehicle_speed;
  const double rear_longitudinal_slip =
      ((kVehicleModel.wheel_radius * rear_wheel_velocity * std::cos(rear_slip_angle)) -
       vehicle_speed) /
      safe_vehicle_speed;
  const double front_lateral_slip =
      (front_wheel_velocity * kVehicleModel.wheel_radius * std::sin(front_slip_angle)) /
      safe_vehicle_speed;
  const double rear_lateral_slip =
      (rear_wheel_velocity * kVehicleModel.wheel_radius * std::sin(rear_slip_angle)) /
      safe_vehicle_speed;

  const double front_slip_magnitude = std::sqrt(
      front_longitudinal_slip * front_longitudinal_slip +
      front_lateral_slip * front_lateral_slip);
  const double rear_slip_magnitude = std::sqrt(
      rear_longitudinal_slip * rear_longitudinal_slip +
      rear_lateral_slip * rear_lateral_slip);

  const double front_normal_force = (kVehicleModel.mass / 2.0) * kVehicleModel.gravity;
  const double rear_normal_force = (kVehicleModel.mass / 2.0) * kVehicleModel.gravity;

  const double front_friction =
      ((kVehicleModel.tire_c1 *
        (1 - std::exp(-kVehicleModel.tire_c2 * front_slip_magnitude))) -
       (kVehicleModel.tire_c3 * front_slip_magnitude)) *
      surface_scale;
  const double rear_friction =
      ((kVehicleModel.tire_c1 *
        (1 - std::exp(-kVehicleModel.tire_c2 * rear_slip_magnitude))) -
       (kVehicleModel.tire_c3 * rear_slip_magnitude)) *
      surface_scale;

  double front_force_x = 0.0;
  double front_force_y = 0.0;
  if (front_slip_magnitude > kSlipForceThreshold) {
    front_force_x =
        (front_friction / front_slip_magnitude) * front_normal_force * front_longitudinal_slip;
    front_force_y =
        (front_friction / front_slip_magnitude) * front_normal_force * front_lateral_slip;
  }

  double rear_force_x = 0.0;
  double rear_force_y = 0.0;
  if (rear_slip_magnitude > kSlipForceThreshold) {
    rear_force_x =
        (rear_friction / rear_slip_magnitude) * rear_normal_force * rear_longitudinal_slip;
    rear_force_y =
        (rear_friction / rear_slip_magnitude) * rear_normal_force * rear_lateral_slip;
  }

  const double x_velocity_derivative =
      ((front_force_x * std::cos(driving_angle)) -
       (front_force_y * std::sin(driving_angle)) + rear_force_x) /
      kVehicleModel.mass;
  const double y_velocity_derivative =
      ((front_force_y * std::cos(driving_angle)) +
       (front_force_x * std::sin(driving_angle)) + rear_force_y) /
      kVehicleModel.mass;

  out[0] = state[0] + x_velocity_derivative;
  out[1] = state[1] + y_velocity_derivative;
  out[2] = state[2];
}

static void finite_difference_jacobian(
    const double state[EKF_N],
    const double input[EKF_M],
    double jacobian[EKF_N * EKF_N])
{
  double baseline[EKF_N];
  state_transition(state, input, baseline);

  for (int column = 0; column < EKF_N; ++column) {
    double perturbed_state[EKF_N] = {state[0], state[1], state[2]};
    perturbed_state[column] += kDiffStep;

    double perturbed_output[EKF_N];
    state_transition(perturbed_state, input, perturbed_output);

    for (int row = 0; row < EKF_N; ++row) {
      jacobian[row * EKF_N + column] =
          (perturbed_output[row] - baseline[row]) / kDiffStep;
    }
  }
}

}  // namespace

void velocity_ekf_reset(VelocityEkf *ekf, double initial_velocity)
{
  if (ekf == nullptr) {
    return;
  }

  const double pdiag[EKF_N] = {1000000.0, 1000000.0, 1000000.0};
  ekf_t tiny_ekf = {};
  ekf_initialize(&tiny_ekf, pdiag);
  store_tiny_ekf(tiny_ekf, ekf);
  initialize_noise(ekf);
  std::memset(ekf->input, 0, sizeof(ekf->input));

  ekf->state[0] = initial_velocity - 0.01;
  ekf->state[1] = 0.01;
  ekf->state[2] = 1.1;
}

double velocity_ekf_step(
    VelocityEkf *ekf,
    double driving_angle,
    double front_wheel_vel,
    double rear_wheel_vel)
{
  if (ekf == nullptr) {
    return 0.0;
  }

  ekf->input[0] = driving_angle;
  ekf->input[1] = front_wheel_vel;
  ekf->input[2] = rear_wheel_vel;

  ekf_t tiny_ekf = make_tiny_ekf(*ekf);

  double current_state[EKF_N] = {
      ekf->state[0],
      ekf->state[1],
      ekf->state[2],
  };
  double predicted_state[EKF_N];
  double jacobian[EKF_N * EKF_N];
  state_transition(current_state, ekf->input, predicted_state);
  finite_difference_jacobian(current_state, ekf->input, jacobian);

  ekf_predict(&tiny_ekf, predicted_state, jacobian, ekf->process_noise);

  const double measurement[EKF_M] = {
      ekf->input[0],
      ekf->input[1],
      ekf->input[2],
  };
  const double measurement_prediction[EKF_M] = {
      ekf->input[0],
      ekf->input[1],
      ekf->input[2],
  };
  const double measurement_jacobian[EKF_M * EKF_N] = {0.0};
  ekf_update(
      &tiny_ekf,
      measurement,
      measurement_prediction,
      measurement_jacobian,
      ekf->measurement_noise);

  store_tiny_ekf(tiny_ekf, ekf);
  return ekf->state[0];
}
