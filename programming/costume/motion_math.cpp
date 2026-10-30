// motion_math.cpp
// Implementation file for ESP32 Feather V2 + LSM6DS3TR-C + LIS3MDL motion math modules

#include "motion_math.h"



// -----------------------------------------------------------------------------
// sensor setup / reading
// -----------------------------------------------------------------------------
bool setup_sensors(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl) {
  bool ok_lsm = lsm6ds.begin_I2C();
  bool ok_mag = lis3mdl.begin_I2C(0x1E);

  if (!ok_lsm) {
    Serial.println("Failed to find LSM6DS3TR-C.");
  }
  if (!ok_mag) {
    Serial.println("Failed to find LIS3MDL 0x1E.");
    ok_mag = lis3mdl.begin_I2C();
    Serial.println("Trying again...");
    if (!ok_mag) {
      Serial.println("Failed to find LIS3MDL");
    }
  }

  return ok_lsm && ok_mag;
}

void configure_sensors(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl) {
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // TODO(calibration): Apply accel zero-offset (bias) correction per axis.
  // Measure mean accel output at rest for each axis and subtract from raw
  // readings in read_imu(). The LSM6DS3TRC has a factory trim but on-board
  // placement introduces an additional fixed offset.
  // lsm6ds.setAccelOffset(x_offset, y_offset, z_offset);

  // TODO(calibration): Apply gyro zero-rate offset (drift at rest).
  // Average ~500 gyro samples with the device perfectly still and store as
  // bias to subtract per-frame. Gyro bias drifts with temperature — recalibrate
  // if the costume sits in the sun or under stage lights.
  // lsm6ds.setGyroOffset(x_offset, y_offset, z_offset);

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // TODO(calibration): Hard-iron offset correction for the magnetometer.
  // Rotate the device through all orientations (figure-8 motion), collect
  // min/max per axis, then offset = (max + min) / 2. Subtract per-frame in
  // read_imu() before using mag data. Re-run whenever the costume changes
  // (zippers, buckles, phone nearby all shift the hard-iron field).

  // TODO(calibration): Soft-iron scale correction for the magnetometer.
  // After hard-iron, scale each axis so the sphere of readings is uniform.
  // A 3x3 scale matrix is needed for full correction; per-axis scale factors
  // are a simpler approximation.

  // TODO(calibration): Magnetic declination for accurate compass heading.
  // Add the local declination angle (degrees) to heading_deg in
  // compute_heading_deg(). Look up value at: https://www.ngdc.noaa.gov/geomag/
}

bool read_imu(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              IMUFrame &frame,
              unsigned long &last_read_ms,
              uint8_t mag_address) {
  (void)mag_address;

  sensors_event_t accel, gyro, temp, mag;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  unsigned long now = millis();
  float dt = (last_read_ms == 0) ? 0.01f : (now - last_read_ms) * 0.001f;
  last_read_ms = now;

  frame.accel = { accel.acceleration.x, accel.acceleration.y, accel.acceleration.z };
  frame.gyro  = { gyro.gyro.x, gyro.gyro.y, gyro.gyro.z };
  frame.mag   = { mag.magnetic.x, mag.magnetic.y, mag.magnetic.z };
  frame.temp_c = temp.temperature;
  frame.t_millis = now;
  frame.dt = dt;

  return true;
}

// -----------------------------------------------------------------------------
// math helpers
// -----------------------------------------------------------------------------
float magnitude(const Vec3 &v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 normalize(const Vec3 &v) {
  float m = magnitude(v);
  if (m < 1e-6f) {
    return {0.0f, 0.0f, 0.0f};
  }

  return { v.x / m, v.y / m, v.z / m };
}

float dot_product(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

float wrap_360(float deg) {
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  return deg;
}

// -----------------------------------------------------------------------------
// orientation
// -----------------------------------------------------------------------------
Orientation compute_orientation(const IMUFrame &frame) {
  Orientation orientation;

  // TODO(calibration): Mount orientation remapping.
  // If the sensor is not mounted with its Z-axis pointing up and X-axis
  // pointing forward, remap frame.accel and frame.gyro axes here before
  // computing roll/pitch. E.g. if the board is mounted sideways:
  //   Vec3 remapped_accel = { frame.accel.y, -frame.accel.x, frame.accel.z };
  // Define the remapping once and apply to both accel and gyro.

  // TODO(calibration): Roll/pitch zero reference offset.
  // When the costume is worn in its natural resting position, measure the
  // mean roll_deg and pitch_deg over ~1 second and store as mount_roll_offset
  // and mount_pitch_offset. Subtract from each result below so that
  // "standing upright in costume" always reads 0° / 0°.

  orientation.roll_deg = atan2f(frame.accel.y, frame.accel.z) * RAD_TO_DEG_F;
  orientation.pitch_deg = atan2f(
    -frame.accel.x,
    sqrtf(frame.accel.y * frame.accel.y + frame.accel.z * frame.accel.z)
  ) * RAD_TO_DEG_F;

  orientation.heading_deg = compute_heading_deg(frame.mag);

  return orientation;
}

float compute_heading_deg(const Vec3 &mag) {
  float heading = atan2f(mag.y, mag.x) * RAD_TO_DEG_F;
  return wrap_360(heading);
}

// -----------------------------------------------------------------------------
// motion feature extraction
// -----------------------------------------------------------------------------
MotionFeatures compute_motion_features(const IMUFrame &frame, DetectorState &state) {
  MotionFeatures features;

  features.accel_magnitude = magnitude(frame.accel);

  float accel_mag_hp = state.accel_mag_hpf.update(features.accel_magnitude);
  features.dynamic_accel_magnitude = fabsf(accel_mag_hp);

  features.gyro_magnitude = magnitude(frame.gyro);

  features.motion_energy = state.motion_energy_lpf.update(features.dynamic_accel_magnitude);

  Vec3 gravity_dir = normalize(frame.accel);
  float vertical_accel = compute_vertical_acceleration(frame.accel, gravity_dir);
  state.vertical_lpf.update(vertical_accel);
  features.vertical_dynamic = state.vertical_hpf.update(vertical_accel);

  return features;
}

float compute_vertical_acceleration(const Vec3 &accel, const Vec3 &gravity_dir) {
  return dot_product(accel, gravity_dir);
}

// -----------------------------------------------------------------------------
// event detection
// -----------------------------------------------------------------------------
DetectionFlags detect_events(const MotionFeatures &features, DetectorState &state) {
  DetectionFlags flags;
  unsigned long now = millis();

  flags.impact = detect_impact(
    features.dynamic_accel_magnitude,
    now,
    state.last_impact_ms,
    2.2f,
    180
  );

  flags.spinning = detect_spin(
    features.gyro_magnitude,
    1.8f
  );

  flags.step_pulse = detect_step_pulse(
    features.vertical_dynamic,
    now,
    state.last_step_ms,
    1.1f,
    220,
    900
  );

  return flags;
}

bool detect_impact(float dynamic_accel_magnitude,
                   unsigned long now_ms,
                   unsigned long &last_trigger_ms,
                   float threshold,
                   unsigned long cooldown_ms) {
  if (dynamic_accel_magnitude > threshold && (now_ms - last_trigger_ms) > cooldown_ms) {
    last_trigger_ms = now_ms;
    return true;
  }
  return false;
}

bool detect_spin(float gyro_magnitude, float threshold) {
  return gyro_magnitude > threshold;
}

bool detect_step_pulse(float vertical_dynamic,
                       unsigned long now_ms,
                       unsigned long &last_step_ms,
                       float threshold,
                       unsigned long min_step_interval_ms,
                       unsigned long max_step_interval_ms) {
  if (vertical_dynamic > threshold) {
    unsigned long dt = now_ms - last_step_ms;

    if (dt > min_step_interval_ms && dt < max_step_interval_ms) {
      last_step_ms = now_ms;
      return true;
    }

    if (last_step_ms == 0 || dt >= max_step_interval_ms) {
      last_step_ms = now_ms;
      return true;
    }
  }

  return false;
}

// -----------------------------------------------------------------------------
// Extended detection
// -----------------------------------------------------------------------------

float compute_tilt_angle_deg(const Orientation &orientation) {
  float roll_rad  = orientation.roll_deg  * DEG_TO_RAD_F;
  float pitch_rad = orientation.pitch_deg * DEG_TO_RAD_F;
  // Great-circle distance from vertical
  return acosf(cosf(roll_rad) * cosf(pitch_rad)) * RAD_TO_DEG_F;
}

JumpFlags detect_jump(float accel_magnitude,
                      float dynamic_accel_magnitude,
                      unsigned long now_ms,
                      DetectorState &state) {
  JumpFlags flags = { false, false };

  const float freefall_threshold   = 2.5f;  // m/s² — near weightlessness
  const float landing_threshold    = 3.0f;  // m/s² dynamic — impact after freefall
  const unsigned long min_freefall_ms = 80; // ignore brief micro-hops below this

  bool currently_falling = accel_magnitude < freefall_threshold;

  if (currently_falling) {
    if (!state.was_in_freefall) {
      state.was_in_freefall  = true;
      state.freefall_start_ms = now_ms;
    }
    flags.in_freefall = (now_ms - state.freefall_start_ms) >= min_freefall_ms;
  } else {
    if (state.was_in_freefall) {
      unsigned long duration = now_ms - state.freefall_start_ms;
      state.was_in_freefall = false;
      state.freefall_end_ms = now_ms;

      if (duration >= min_freefall_ms && dynamic_accel_magnitude > landing_threshold) {
        flags.just_landed = true;
      }
    }
  }

  return flags;
}

PostureFlags detect_posture(const Orientation &orientation,
                            const MotionFeatures &features,
                            unsigned long now_ms,
                            DetectorState &state) {
  PostureFlags flags = { false, false, false, false, false };

  float abs_roll  = fabsf(orientation.roll_deg);
  float abs_pitch = fabsf(orientation.pitch_deg);

  flags.upright         = abs_roll < 20.0f && abs_pitch < 20.0f;
  flags.lying_down      = abs_roll > 60.0f || abs_pitch > 60.0f;
  flags.tilted_forward  = orientation.pitch_deg >  45.0f;
  flags.tilted_backward = orientation.pitch_deg < -45.0f;

  // TODO(calibration): All angle thresholds above assume 0° roll and 0° pitch
  // equals the natural upright worn position. Once mount_roll_offset and
  // mount_pitch_offset are applied in compute_orientation(), these thresholds
  // should be correct. Without that offset, adjust the constants here to match
  // the actual resting readings from the Serial monitor.

  // Crouch detection via vertical_dynamic signature 
  // A crouch produces a downward burst (negative vertical_dynamic) followed
  // by settling to near-zero within a short window, while remaining upright.
  const float down_threshold  = -0.8f;  // downward burst to trigger phase 1
  const float settle_threshold = 0.2f;  // absolute value — "body has settled"
  const unsigned long crouch_window_ms = 600; // max ms between burst and settle

  if (flags.upright && !flags.lying_down) {
    if (features.vertical_dynamic < down_threshold) {
      // Phase 1: caught the downward movement
      state.crouch_down_seen = true;
      state.crouch_down_ms   = now_ms;
    }

    if (state.crouch_down_seen) {
      unsigned long elapsed = now_ms - state.crouch_down_ms;

      if (elapsed <= crouch_window_ms &&
          fabsf(features.vertical_dynamic) < settle_threshold &&
          features.motion_energy < 0.3f) {
        // Phase 2: body has settled after going down — crouching.
        // motion_energy guard rules out stumbles/trips where the body
        // is still lurching after the downward burst.
        flags.crouching = true;
      } else if (elapsed > crouch_window_ms) {
        // Too long without settling — reset, was probably a step or stumble
        state.crouch_down_seen = false;
      }
    }
  } else {
    // Not upright — reset crouch state
    state.crouch_down_seen = false;
  }

  return flags;
}

EmergencyFlags detect_emergency(const MotionFeatures &features,
                                const PostureFlags &posture,
                                const JumpFlags &jump,
                                unsigned long now_ms,
                                DetectorState &state) {
  EmergencyFlags flags = { false, false, false, false };

  const float active_threshold        = 0.15f;  // motion_energy above this = active
  const float still_threshold         = 0.04f;  // motion_energy below this = still
  const float sudden_drop_ratio       = 0.15f;  // energy dropped to <15% of prev frame
  const unsigned long faint_duration_ms = 5000; // still + lying for this long = suspected faint
  const unsigned long freefall_emergency_ms = 600; // freefall longer than this = emergency

  // Sustained freefall
  if (state.was_in_freefall &&
      (now_ms - state.freefall_start_ms) > freefall_emergency_ms) {
    flags.freefall_sustained = true;
  }

  // Hard landing: large impact right after freefall
  flags.hard_landing = jump.just_landed;

  // Track last time the wearer was active
  if (features.motion_energy > active_threshold) {
    state.last_active_ms = now_ms;
  }

  // Sudden stillness: energy collapsed rapidly from a previously moving state
  if (state.prev_motion_energy > active_threshold &&
      features.motion_energy < state.prev_motion_energy * sudden_drop_ratio) {
    flags.sudden_stillness = true;
  }
  state.prev_motion_energy = features.motion_energy;

  // Faint: lying down, currently still, and has been still for a prolonged period
  bool prolonged_stillness = (state.last_active_ms > 0) &&
                             (now_ms - state.last_active_ms) > faint_duration_ms;
  if (posture.lying_down &&
      features.motion_energy < still_threshold &&
      prolonged_stillness) {
    flags.faint_suspected = true;
  }

  return flags;
}

ShakeFlags detect_shake(float dynamic_accel_magnitude, DetectorState &state) {
  ShakeFlags flags = { false };

  const float shake_threshold = 0.8f; // smoothed dynamic energy level indicating shake

  float shake_energy = state.shake_energy_lpf.update(dynamic_accel_magnitude);
  flags.shaking = shake_energy > shake_threshold;

  return flags;
}

// -----------------------------------------------------------------------------
// debug / serial output
// -----------------------------------------------------------------------------
void print_frame(const IMUFrame &frame,
                 const Orientation &orientation,
                 const MotionFeatures &features,
                 const DetectionFlags &flags,
                 const JumpFlags &jump,
                 const PostureFlags &posture,
                 const EmergencyFlags &emergency,
                 const ShakeFlags &shake,
                 float tilt_angle_deg) {
  Serial.println("--------------------------------------------------");

  Serial.print("Accel (m/s^2): ");
  Serial.print(frame.accel.x, 3);
  Serial.print(", ");
  Serial.print(frame.accel.y, 3);
  Serial.print(", ");
  Serial.println(frame.accel.z, 3);

  Serial.print("Gyro (rad/s): ");
  Serial.print(frame.gyro.x, 3);
  Serial.print(", ");
  Serial.print(frame.gyro.y, 3);
  Serial.print(", ");
  Serial.println(frame.gyro.z, 3);

  Serial.print("Mag (uT): ");
  Serial.print(frame.mag.x, 3);
  Serial.print(", ");
  Serial.print(frame.mag.y, 3);
  Serial.print(", ");
  Serial.println(frame.mag.z, 3);

  Serial.print("Temp (C): ");
  Serial.println(frame.temp_c, 2);

  Serial.print("Roll (deg): ");
  Serial.println(orientation.roll_deg, 2);

  Serial.print("Pitch (deg): ");
  Serial.println(orientation.pitch_deg, 2);

  Serial.print("Heading (deg): ");
  Serial.println(orientation.heading_deg, 2);

  Serial.print("Tilt angle (deg): ");
  Serial.println(tilt_angle_deg, 2);

  Serial.print("Accel magnitude: ");
  Serial.println(features.accel_magnitude, 3);

  Serial.print("Dynamic accel magnitude: ");
  Serial.println(features.dynamic_accel_magnitude, 3);

  Serial.print("Gyro magnitude: ");
  Serial.println(features.gyro_magnitude, 3);

  Serial.print("Motion energy: ");
  Serial.println(features.motion_energy, 3);

  Serial.print("Vertical dynamic: ");
  Serial.println(features.vertical_dynamic, 3);

  Serial.print("Impact: ");
  Serial.print(flags.impact ? "YES" : "NO");
  Serial.print("  |  Spinning: ");
  Serial.print(flags.spinning ? "YES" : "NO");
  Serial.print("  |  Step: ");
  Serial.println(flags.step_pulse ? "YES" : "NO");

  Serial.print("Freefall: ");
  Serial.print(jump.in_freefall ? "YES" : "NO");
  Serial.print("  |  Landed: ");
  Serial.println(jump.just_landed ? "YES" : "NO");

  Serial.print("Posture — Upright: ");
  Serial.print(posture.upright ? "YES" : "NO");
  Serial.print("  |  Crouching: ");
  Serial.print(posture.crouching ? "YES" : "NO");
  Serial.print("  |  Lying down: ");
  Serial.print(posture.lying_down ? "YES" : "NO");
  Serial.print("  |  Fwd: ");
  Serial.print(posture.tilted_forward ? "YES" : "NO");
  Serial.print("  |  Bwd: ");
  Serial.println(posture.tilted_backward ? "YES" : "NO");

  Serial.print("Shaking: ");
  Serial.println(shake.shaking ? "YES" : "NO");

  Serial.print("Emergency — Freefall: ");
  Serial.print(emergency.freefall_sustained ? "YES" : "NO");
  Serial.print("  |  Hard landing: ");
  Serial.print(emergency.hard_landing ? "YES" : "NO");
  Serial.print("  |  Sudden stop: ");
  Serial.print(emergency.sudden_stillness ? "YES" : "NO");
  Serial.print("  |  Faint: ");
  Serial.println(emergency.faint_suspected ? "YES" : "NO");

  Serial.println();
}
