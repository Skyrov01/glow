// motion_math.h

#ifndef MOTION_MATH_H
#define MOTION_MATH_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <math.h>
#include "filters.h"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
static constexpr float GRAVITY_MPS2 = 9.80665f;
static constexpr float RAD_TO_DEG_F = 57.2957795f;
static constexpr float DEG_TO_RAD_F = 0.0174532925f;

// -----------------------------------------------------------------------------
// Basic vector types
// -----------------------------------------------------------------------------
struct Vec3 {
  float x;
  float y;
  float z;
};

struct IMUFrame {
  Vec3 accel;
  Vec3 gyro;
  Vec3 mag;
  float temp_c;
  unsigned long t_millis;
  float dt;
};

struct Orientation {
  float roll_deg;
  float pitch_deg;
  float heading_deg;
};

struct MotionFeatures {
  float accel_magnitude;
  float dynamic_accel_magnitude;
  float gyro_magnitude;
  float motion_energy;
  float vertical_dynamic;
};

struct DetectionFlags {
  bool impact;
  bool spinning;
  bool step_pulse;
};

struct JumpFlags {
  bool in_freefall;     // accel magnitude near 0g (< ~2 m/s²)
  bool just_landed;     // impact immediately following freefall
};

struct PostureFlags {
  bool upright;          // roll and pitch both within ~20° of vertical
  bool crouching;        // upright but accel.z significantly reduced
  bool lying_down;       // pitch or roll > 60° from vertical
  bool tilted_forward;   // pitch > 45°
  bool tilted_backward;  // pitch < -45°
};

struct EmergencyFlags {
  bool freefall_sustained;   // near-zero gravity for > 300 ms
  bool hard_landing;         // large impact immediately after freefall
  bool sudden_stillness;     // motion energy collapsed rapidly (abrupt stop)
  bool faint_suspected;      // lying_down + sudden_stillness + prolonged stillness
};

struct ShakeFlags {
  bool shaking;              // rapid back-and-forth accel oscillation
};

// -----------------------------------------------------------------------------
// Detector state
// -----------------------------------------------------------------------------
struct DetectorState {
  LowPassFilter accel_mag_lpf;
  HighPassFilter accel_mag_hpf;
  LowPassFilter motion_energy_lpf;
  LowPassFilter vertical_lpf;
  HighPassFilter vertical_hpf;

  unsigned long last_impact_ms;
  unsigned long last_step_ms;

  // Jump / freefall state
  bool was_in_freefall;
  unsigned long freefall_start_ms;
  unsigned long freefall_end_ms;

  // Emergency / faint state
  unsigned long last_active_ms;     // last time motion_energy was above idle threshold
  float prev_motion_energy;

  // Shake state
  LowPassFilter shake_energy_lpf;

  // Crouch state
  bool crouch_down_seen;          // negative vertical_dynamic burst detected
  unsigned long crouch_down_ms;   // when the downward burst was seen

  DetectorState()
    : accel_mag_lpf(0.08f),
      accel_mag_hpf(0.92f),
      motion_energy_lpf(0.08f),
      vertical_lpf(0.10f),
      vertical_hpf(0.90f),
      last_impact_ms(0),
      last_step_ms(0),
      was_in_freefall(false),
      freefall_start_ms(0),
      freefall_end_ms(0),
      last_active_ms(0),
      prev_motion_energy(0.0f),
      shake_energy_lpf(0.15f),
      crouch_down_seen(false),
      crouch_down_ms(0) {}
};

bool setup_sensors(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl);
void configure_sensors(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl);
bool read_imu(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              IMUFrame &frame,
              unsigned long &last_read_ms,
              uint8_t mag_address = 0x1E);

float magnitude(const Vec3 &v);
Vec3 normalize(const Vec3 &v);
float dot_product(const Vec3 &a, const Vec3 &b);
float wrap_360(float deg);

Orientation compute_orientation(const IMUFrame &frame);
float compute_heading_deg(const Vec3 &mag);

MotionFeatures compute_motion_features(const IMUFrame &frame, DetectorState &state);
float compute_vertical_acceleration(const Vec3 &accel, const Vec3 &gravity_dir);

DetectionFlags detect_events(const MotionFeatures &features, DetectorState &state);

bool detect_impact(float dynamic_accel_magnitude,
                   unsigned long now_ms,
                   unsigned long &last_trigger_ms,
                   float threshold,
                   unsigned long cooldown_ms);

bool detect_spin(float gyro_magnitude, float threshold);

bool detect_step_pulse(float vertical_dynamic,
                       unsigned long now_ms,
                       unsigned long &last_step_ms,
                       float threshold,
                       unsigned long min_step_interval_ms,
                       unsigned long max_step_interval_ms);

void print_frame(const IMUFrame &frame,
                 const Orientation &orientation,
                 const MotionFeatures &features,
                 const DetectionFlags &flags,
                 const JumpFlags &jump,
                 const PostureFlags &posture,
                 const EmergencyFlags &emergency,
                 const ShakeFlags &shake,
                 float tilt_angle_deg);

// -----------------------------------------------------------------------------
// Extended detection
// -----------------------------------------------------------------------------

// Returns how many degrees the device is tilted away from vertical (0 = upright).
float compute_tilt_angle_deg(const Orientation &orientation);

// Detects jump phases: freefall (accel near 0) and landing (impact after freefall).
JumpFlags detect_jump(float accel_magnitude,
                      float dynamic_accel_magnitude,
                      unsigned long now_ms,
                      DetectorState &state);

// Classifies body posture based on orientation angles.
PostureFlags detect_posture(const Orientation &orientation,
                            const MotionFeatures &features,
                            unsigned long now_ms,
                            DetectorState &state);

// Detects emergency situations: sustained freefall, hard landing, faint.
// Call after detect_jump() and detect_posture() so their results are available.
EmergencyFlags detect_emergency(const MotionFeatures &features,
                                const PostureFlags &posture,
                                const JumpFlags &jump,
                                unsigned long now_ms,
                                DetectorState &state);

// Detects rapid back-and-forth shaking motion.
ShakeFlags detect_shake(float dynamic_accel_magnitude,
                        DetectorState &state);

#endif // MOTION_MATH_H
