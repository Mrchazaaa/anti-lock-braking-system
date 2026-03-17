#include "ABS/abs.h"

#include <cmath>

#include "velocity_ekf.h"

namespace {

constexpr float kMinMagnitude = 1e-6f;
constexpr float kInactiveTimer = -1.0f;
constexpr float kAggressiveRecoveryFactor = 10.0f;

const AbsConfig kConfig = {
    10.0f,
    10.0f,
    13000.0f,
    0.04f,
    11000000.0f,
    8458000.0f,
    50000000.0f,
    -95.0f,
    0.0f,
    0.12f,
    13000000.0f,
    0.3179f,
};

enum class Phase : int {
  kOff = 0,
  kBuildPressure = 1,
  kMonitorSlip = 2,
  kReleasePressure = 3,
  kHoldAfterRelease = 4,
  kPrimaryReapply = 5,
  kHoldAfterPrimaryReapply = 6,
  kSecondaryReapply = 7,
};

struct WheelState {
  Phase phase = Phase::kOff;
  float timer = kInactiveTimer;
  float spin_velocity = 0.0f;
  float spin_acceleration = 0.0f;
  float slip_acceleration = 0.0f;
  float slip = 0.0f;
  float max_slip = kConfig.initial_max_wheel_slip;
  float brake_command = 0.0f;
};

struct ControllerRuntime {
  float last_timestamp = 0.0f;
  float delta_time = 0.0f;
  float vehicle_speed = 0.0f;
  bool ekf_active = false;
};

static float clamp_nonzero(float value)
{
  if (std::fabs(value) >= kMinMagnitude) {
    return value;
  }
  return value < 0.0f ? -kMinMagnitude : kMinMagnitude;
}

static float pressure_to_command(float delta_time, float pressure_rate)
{
  return (delta_time * pressure_rate) / kConfig.max_brake_pressure;
}

static bool update_delay_timer(float *timer, float delta_time)
{
  if (*timer < 0.0f) {
    *timer = kConfig.apply_delay;
  } else {
    *timer -= delta_time;
  }

  return *timer < 0.0f;
}

static float compute_average_front_wheel_speed(const AbsStepInput &input)
{
  return (input.wheel_spin_velocity[ABS_FL] + input.wheel_spin_velocity[ABS_FR]) / 2.0f;
}

static float compute_average_rear_wheel_speed(const AbsStepInput &input)
{
  return (input.wheel_spin_velocity[ABS_RL] + input.wheel_spin_velocity[ABS_RR]) / 2.0f;
}

static void reset_wheel_state(WheelState *wheel)
{
  wheel->phase = Phase::kOff;
  wheel->timer = kInactiveTimer;
  wheel->spin_velocity = 0.0f;
  wheel->spin_acceleration = 0.0f;
  wheel->slip_acceleration = 0.0f;
  wheel->slip = 0.0f;
  wheel->max_slip = kConfig.initial_max_wheel_slip;
  wheel->brake_command = 0.0f;
}

static void deactivate_wheel(WheelState *wheel, float requested_pressure, bool passthrough_brake)
{
  wheel->phase = Phase::kOff;
  wheel->timer = kInactiveTimer;
  wheel->max_slip = kConfig.initial_max_wheel_slip;
  if (passthrough_brake) {
    wheel->brake_command = requested_pressure;
  }
}

static void update_phase(
    WheelState *wheel,
    float delta_time,
    float requested_pressure)
{
  switch (wheel->phase) {
    case Phase::kOff:
      wheel->phase = Phase::kBuildPressure;
      break;
    case Phase::kBuildPressure:
      wheel->brake_command = requested_pressure;
      if (wheel->spin_acceleration < kConfig.min_wheel_spin_acceleration) {
        wheel->phase = Phase::kMonitorSlip;
      }
      break;
    case Phase::kMonitorSlip:
      if (wheel->slip > wheel->max_slip) {
        wheel->max_slip = wheel->slip;
        wheel->phase = Phase::kReleasePressure;
      }
      break;
    case Phase::kReleasePressure:
      wheel->brake_command -= pressure_to_command(delta_time, kConfig.release_rate);
      if (wheel->spin_acceleration > kConfig.max_wheel_spin_acceleration) {
        wheel->phase = Phase::kHoldAfterRelease;
      }
      break;
    case Phase::kHoldAfterRelease:
      if (update_delay_timer(&wheel->timer, delta_time) ||
          wheel->spin_acceleration >
              (kConfig.max_wheel_spin_acceleration * kAggressiveRecoveryFactor)) {
        wheel->timer = kInactiveTimer;
        wheel->phase = Phase::kPrimaryReapply;
      }
      break;
    case Phase::kPrimaryReapply:
      wheel->brake_command += pressure_to_command(delta_time, kConfig.primary_apply_rate);
      if (wheel->spin_acceleration < 0.0f) {
        wheel->phase = Phase::kHoldAfterPrimaryReapply;
      }
      break;
    case Phase::kHoldAfterPrimaryReapply:
      if (update_delay_timer(&wheel->timer, delta_time) ||
          wheel->spin_acceleration < kConfig.min_wheel_spin_acceleration) {
        wheel->timer = kInactiveTimer;
        wheel->phase = Phase::kSecondaryReapply;
      }
      break;
    case Phase::kSecondaryReapply:
      if (wheel->brake_command >= 1.0f) {
        wheel->brake_command = 1.0f;
      } else {
        wheel->brake_command += pressure_to_command(delta_time, kConfig.secondary_apply_rate);
      }
      if (wheel->spin_acceleration < kConfig.min_wheel_spin_acceleration) {
        wheel->phase = Phase::kReleasePressure;
      }
      break;
  }
}

}  // namespace

struct AbsController {
  WheelState wheels[ABS_WHEEL_COUNT];
  ControllerRuntime runtime;
  VelocityEkf ekf;
};

static void reset_runtime_state(AbsController *controller)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    reset_wheel_state(&controller->wheels[i]);
  }
  controller->runtime = {};
  velocity_ekf_reset(&controller->ekf, 0.0);
}

static void update_timestep(AbsController *controller, float timestamp)
{
  if (controller->runtime.last_timestamp == 0.0f) {
    controller->runtime.ekf_active = false;
    controller->runtime.last_timestamp = timestamp;
  }

  controller->runtime.delta_time = timestamp - controller->runtime.last_timestamp;
  if (controller->runtime.delta_time <= 0.0f) {
    controller->runtime.delta_time = kMinMagnitude;
  }
}

static void update_wheel_kinematics(
    AbsController *controller,
    const AbsStepInput *input)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    WheelState &wheel = controller->wheels[i];
    wheel.spin_acceleration =
        (input->wheel_spin_velocity[i] - wheel.spin_velocity) /
        controller->runtime.delta_time;
    wheel.spin_velocity = input->wheel_spin_velocity[i];
  }
}

static void update_vehicle_speed(
    AbsController *controller,
    const AbsStepInput *input)
{
  if (!controller->runtime.ekf_active) {
    controller->runtime.vehicle_speed =
        compute_average_front_wheel_speed(*input) * kConfig.wheel_radius_static;
    return;
  }

  controller->runtime.vehicle_speed = static_cast<float>(velocity_ekf_step(
      &controller->ekf,
      0.0,
      compute_average_front_wheel_speed(*input),
      compute_average_rear_wheel_speed(*input)));
}

static void update_wheel_slip(AbsController *controller)
{
  const float safe_vehicle_speed = clamp_nonzero(controller->runtime.vehicle_speed);
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    WheelState &wheel = controller->wheels[i];
    const float next_slip =
        (controller->runtime.vehicle_speed -
         wheel.spin_velocity * kConfig.wheel_radius_static) /
        safe_vehicle_speed;
    wheel.slip_acceleration =
        (next_slip - wheel.slip) / controller->runtime.delta_time;
    wheel.slip = next_slip;
  }
}

static bool abs_control_enabled(const AbsController *controller, const AbsStepInput *input)
{
  return controller->runtime.vehicle_speed > kConfig.min_vehicle_velocity_threshold &&
         input->requested_pressure >
             (kConfig.min_pressure_threshold / kConfig.max_brake_pressure);
}

static bool wheel_is_eligible(const WheelState &wheel)
{
  return wheel.spin_velocity * kConfig.wheel_radius_static >
         kConfig.min_wheel_velocity_threshold;
}

static void update_wheel_controller(
    AbsController *controller,
    int wheel_index,
    float requested_pressure)
{
  WheelState &wheel = controller->wheels[wheel_index];
  update_phase(&wheel, controller->runtime.delta_time, requested_pressure);

  if (!controller->runtime.ekf_active) {
    velocity_ekf_reset(&controller->ekf, controller->runtime.vehicle_speed);
    controller->runtime.ekf_active = true;
  }
}

static void run_abs_control(
    AbsController *controller,
    const AbsStepInput *input)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    WheelState &wheel = controller->wheels[i];
    if (wheel_is_eligible(wheel)) {
      update_wheel_controller(controller, i, input->requested_pressure);
    } else {
      deactivate_wheel(&wheel, input->requested_pressure, false);
    }
  }
}

static void run_passthrough_braking(
    AbsController *controller,
    const AbsStepInput *input)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    deactivate_wheel(&controller->wheels[i], input->requested_pressure, true);
  }
  controller->runtime.last_timestamp = 0.0f;
}

static void copy_debug_state(
    const AbsController *controller,
    AbsStepOutput *output)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    const WheelState &wheel = controller->wheels[i];
    output->brake_command[i] = wheel.brake_command;
    output->debug.phase_states[i] = static_cast<int>(wheel.phase);
    output->debug.wheel_spin_velocity[i] = wheel.spin_velocity;
    output->debug.wheel_spin_acceleration[i] = wheel.spin_acceleration;
    output->debug.wheel_slip_acceleration[i] = wheel.slip_acceleration;
    output->debug.wheel_slip[i] = wheel.slip;
    output->debug.max_wheel_slip[i] = wheel.max_slip;
  }
  output->debug.delta_time = controller->runtime.delta_time;
  output->debug.vehicle_speed = controller->runtime.vehicle_speed;
}

const AbsConfig *abs_get_config(void)
{
  return &kConfig;
}

AbsController *abs_create(void)
{
  AbsController *controller = new AbsController();
  reset_runtime_state(controller);
  return controller;
}

void abs_destroy(AbsController *controller)
{
  delete controller;
}

void abs_reset(AbsController *controller)
{
  if (controller == nullptr) {
    return;
  }
  reset_runtime_state(controller);
}

void abs_step(
    AbsController *controller,
    const AbsStepInput *input,
    AbsStepOutput *output)
{
  if (controller == nullptr || input == nullptr || output == nullptr) {
    return;
  }

  update_timestep(controller, input->timestamp);
  update_wheel_kinematics(controller, input);
  update_vehicle_speed(controller, input);
  update_wheel_slip(controller);
  controller->runtime.last_timestamp = input->timestamp;

  if (abs_control_enabled(controller, input)) {
    run_abs_control(controller, input);
  } else {
    run_passthrough_braking(controller, input);
  }

  copy_debug_state(controller, output);
}
