// costume.ino
// Main sketch using:
//   - motion_math.h / motion_math.cpp
//   - microphone_math.h / microphone_math.cpp
//
// Put these 5 files in the same Arduino sketch folder:
//   - costume.ino
//   - motion_math.h
//   - motion_math.cpp
//   - microphone_math.h
//   - microphone_math.cpp

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <esp32-hal-timer.h>
#include "motion_math.h"
#include "microphone_math.h"
#include "costume_lights.h"
#include "analog_sensors.h"

// -----------------------------------------------------------------------------
// Hardware timer for 1 Hz debug output
// -----------------------------------------------------------------------------
hw_timer_t *debug_timer = nullptr;
volatile bool debug_print_tick = false;

void IRAM_ATTR on_debug_timer() {
  debug_print_tick = true;
}

// -----------------------------------------------------------------------------
// Global sensor objects
// -----------------------------------------------------------------------------
Adafruit_LSM6DS3TRC lsm6ds;
Adafruit_LIS3MDL lis3mdl;

// -----------------------------------------------------------------------------
// Microphone pin configuration
// Update these to your actual wiring
// -----------------------------------------------------------------------------
static const I2SMicPins mic_pins = {
  .bclk_pin = 5,
  .ws_pin = 33,
  .data_pin = 21
};

// -----------------------------------------------------------------------------
// Motion state
// -----------------------------------------------------------------------------
DetectorState motion_detector_state;
unsigned long last_motion_read_ms = 0;

IMUFrame current_imu_frame;
Orientation current_orientation;
MotionFeatures current_motion_features;
DetectionFlags current_motion_flags;
JumpFlags current_jump_flags;
PostureFlags current_posture_flags;
EmergencyFlags current_emergency_flags;
ShakeFlags current_shake_flags;
float current_tilt_angle_deg = 0.0f;

// -----------------------------------------------------------------------------
// Audio state
// -----------------------------------------------------------------------------
AudioDetectorState audio_detector_state;
unsigned long last_audio_read_ms = 0;

AudioFrame current_audio_frame;
AudioFeatures current_audio_features;
AudioFlags current_audio_flags;


// -----------------------------------------------------------------------------
// Analog sensors
// -----------------------------------------------------------------------------
AnalogSensorPins analog_pins = {
  .pulse_pin = A3,
  .light_pin = A2
};

AnalogSensorState analog_sensor_state;
unsigned long last_analog_read_ms = 0;

AnalogSensorFrame current_analog_frame;
AnalogSensorFeatures current_analog_features;
AnalogSensorFlags current_analog_flags;



// -----------------------------------------------------------------------------
// Costume Lights
// -----------------------------------------------------------------------------

CostumeLightsConfig lights_config;
CostumeLights costume_lights(lights_config);

uint16_t chase_step = 0;


// -----------------------------------------------------------------------------
// Helper declarations
// -----------------------------------------------------------------------------
float map_to_unit(float value, float min_input, float max_input);

void print_motion_compact_status(const Orientation &orientation,
                                 const MotionFeatures &features,
                                 const DetectionFlags &flags);

void print_audio_compact_status(const AudioFeatures &features,
                                const AudioFlags &flags);

const char *get_dominant_audio_side(const AudioFeatures &features);



// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println(" Costume system starting...");
  Serial.println("========================================");
  Serial.println();
  delay(500);

  // ---------------------------------------------------------------------------
  // Setup IMU
  // ---------------------------------------------------------------------------
  Serial.println("[1/4] Initializing motion sensors...");

  if (!setup_sensors(lsm6ds, lis3mdl)) {
    Serial.println("  ERROR: Motion sensor initialization failed. Halting.");
    while (1) {
      delay(10);
    }
  }

  Serial.println("  Sensors found. Configuring...");
  configure_sensors(lsm6ds, lis3mdl);
  Serial.println("  Motion sensors ready.");

  // TODO(calibration): Run a startup calibration routine here while the
  // wearer stands still for 2-3 seconds after power-on. Collect:
  //   - Gyro bias (mean gyro readings at rest)
  //   - Accel bias (deviation from expected gravity vector per axis)
  //   - Roll/pitch mount offset (mean orientation at natural standing posture)
  // Store results in global variables and apply them inside read_imu() and
  // compute_orientation(). A simple approach: blink the LEDs to signal
  // "hold still", sample for 2 s, then blink again to confirm done.

  delay(500);
  Serial.println();

  // ---------------------------------------------------------------------------
  // Setup microphones
  // ---------------------------------------------------------------------------
  Serial.println("[2/4] Initializing microphones...");

  if (!setup_microphones(mic_pins, I2S_NUM_0)) {
    Serial.println("  ERROR: Microphone initialization failed. Halting.");
    while (1) {
      delay(10);
    }
  }

  Serial.println("  Microphones ready.");
  delay(500);
  Serial.println();

  // ---------------------------------------------------------------------------
  // Setup analog sensors
  // ---------------------------------------------------------------------------
  Serial.println("[3/4] Initializing analog sensors...");

  if (!setup_analog_sensors(analog_pins)) {
    Serial.println("  ERROR: Analog sensor initialization failed. Halting.");
    while (1) {
      delay(10);
    }
  }

  Serial.println("  Analog sensors ready.");
  delay(500);
  Serial.println();

  // ---------------------------------------------------------------------------
  // Setup lights
  // ---------------------------------------------------------------------------
  Serial.println("[4/4] Initializing lights...");

  lights_config.led_pin = 27;
  lights_config.led_count = 10;

  costume_lights.begin();
  costume_lights.set_brightness(125);

  costume_lights.fill(CostumeColors::green);
  delay(250);
  costume_lights.fill(CostumeColors::blue);
  delay(250);

  Serial.println("  Lights ready.");
  delay(500);
  Serial.println();

  // ---------------------------------------------------------------------------
  // Setup timer interrupt for periodic printing
  // ---------------------------------------------------------------------------
  debug_timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
  timerAttachInterrupt(debug_timer, &on_debug_timer, true);
  timerAlarmWrite(debug_timer, 1000000, true); // 1 second interval
  timerAlarmEnable(debug_timer);
  Serial.println("  Timer interrupt for periodic printing enabled.");
  delay(500);
  Serial.println();

  Serial.println("========================================");
  Serial.println(" All systems ready. Starting main loop.");
  Serial.println("========================================");
  Serial.println();

  delay(1500);
}




// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  // ---------------------------------------------------------------------------
  // 1. Read and process IMU
  // ---------------------------------------------------------------------------
  if (read_imu(lsm6ds, lis3mdl, current_imu_frame, last_motion_read_ms, 0x1E)) {
    current_orientation      = compute_orientation(current_imu_frame);
    current_motion_features  = compute_motion_features(current_imu_frame, motion_detector_state);
    current_motion_flags     = detect_events(current_motion_features, motion_detector_state);

    unsigned long now_ms     = millis();
    current_tilt_angle_deg   = compute_tilt_angle_deg(current_orientation);
    current_jump_flags       = detect_jump(current_motion_features.accel_magnitude,
                                           current_motion_features.dynamic_accel_magnitude,
                                           now_ms,
                                           motion_detector_state);
    current_posture_flags    = detect_posture(current_orientation, current_motion_features,
                                               now_ms, motion_detector_state);
    current_emergency_flags  = detect_emergency(current_motion_features,
                                                current_posture_flags,
                                                current_jump_flags,
                                                now_ms,
                                                motion_detector_state);
    current_shake_flags      = detect_shake(current_motion_features.dynamic_accel_magnitude,
                                            motion_detector_state);
  }

  // ---------------------------------------------------------------------------
  // 2. Read and process microphones
  // ---------------------------------------------------------------------------
  if (read_audio_frame(current_audio_frame, last_audio_read_ms, I2S_NUM_0)) {
    current_audio_features = compute_audio_features(current_audio_frame, audio_detector_state);
    current_audio_flags = detect_audio_events(current_audio_features, audio_detector_state);
  }

  // ---------------------------------------------------------------------------
  // 3. Read and process analog sensor
  // ---------------------------------------------------------------------------
  if (read_analog_sensors(current_analog_frame, last_analog_read_ms, analog_pins)) {
    current_analog_features = compute_analog_features(current_analog_frame, analog_sensor_state);
    current_analog_flags = detect_analog_events(current_analog_features, analog_sensor_state);
  }

  // ---------------------------------------------------------------------------
  // 4. Debug output at 1 Hz via hardware timer
  // ---------------------------------------------------------------------------
//  if (debug_print_tick) {
//    debug_print_tick = false;
//
//    if (idle_state) {
//      costume_lights.fill(CostumeColors::blue);
//    } else if (active_state) {
//      costume_lights.fill(CostumeColors::green);
//    } else {
//      costume_lights.fill(CostumeColors::red);
//    }
//
//    print_motion_compact_status(current_orientation, current_motion_features, current_motion_flags);
//    print_audio_compact_status(current_audio_features, current_audio_flags);
//
//    print_frame(current_imu_frame, current_orientation, current_motion_features, current_motion_flags,
//                current_jump_flags, current_posture_flags, current_emergency_flags, current_shake_flags,
//                current_tilt_angle_deg);
//
//    print_analog_frame(current_analog_frame, current_analog_features, current_analog_flags);
//  }

  // ---------------------------------------------------------------------------
  // 5. Placeholder variables for future costume behavior
  // ---------------------------------------------------------------------------

  // Motion-based controls
  float motion_level_01 = map_to_unit(current_motion_features.motion_energy, 0.0f, 3.0f);
  float tilt_level_01   = map_to_unit(current_tilt_angle_deg, 0.0f, 60.0f);

  bool idle_state    = current_motion_features.motion_energy < 0.15f;
  bool active_state  = current_motion_features.motion_energy >= 0.15f &&
                       current_motion_features.motion_energy < 1.0f;
  bool intense_state = current_motion_features.motion_energy >= 1.0f;

  bool leaning_left     = current_orientation.roll_deg < -15.0f;
  bool leaning_right    = current_orientation.roll_deg > 15.0f;
  bool leaning_forward  = current_orientation.pitch_deg > 15.0f;
  bool leaning_backward = current_orientation.pitch_deg < -15.0f;

  // Extended motion
  bool in_freefall   = current_jump_flags.in_freefall;
  bool just_landed   = current_jump_flags.just_landed;
  bool is_shaking    = current_shake_flags.shaking;
  bool is_upright    = current_posture_flags.upright;
  bool is_lying_down = current_posture_flags.lying_down;
  bool is_crouching  = current_posture_flags.crouching;

  // Emergency
  bool emergency_freefall  = current_emergency_flags.freefall_sustained;
  bool emergency_landing   = current_emergency_flags.hard_landing;
  bool emergency_stillness = current_emergency_flags.sudden_stillness;
  bool emergency_faint     = current_emergency_flags.faint_suspected;

  // Audio-based controls
  float sound_level_01 = map_to_unit(current_audio_features.mono_envelope, 0.0f, 0.10f);
  float stereo_balance_01 = map_to_unit(current_audio_features.stereo_difference, 0.0f, 0.05f);

  bool quiet_audio = current_audio_features.mono_envelope < 0.01f;
  bool moderate_audio = current_audio_features.mono_envelope >= 0.01f &&
                        current_audio_features.mono_envelope < 0.05f;
  bool strong_audio = current_audio_features.mono_envelope >= 0.05f;

  bool left_audio_bias =
    current_audio_features.left_envelope > current_audio_features.right_envelope + 0.01f;
  bool right_audio_bias =
    current_audio_features.right_envelope > current_audio_features.left_envelope + 0.01f;

  (void)motion_level_01;
  (void)tilt_level_01;
  (void)idle_state;
  (void)active_state;
  (void)intense_state;
  (void)leaning_left;
  (void)leaning_right;
  (void)leaning_forward;
  (void)leaning_backward;
  (void)in_freefall;
  (void)just_landed;
  (void)is_shaking;
  (void)is_upright;
  (void)is_lying_down;
  (void)is_crouching;
  (void)emergency_freefall;
  (void)emergency_landing;
  (void)emergency_stillness;
  (void)emergency_faint;

  (void)sound_level_01;
  (void)stereo_balance_01;
  (void)quiet_audio;
  (void)moderate_audio;
  (void)strong_audio;
  (void)left_audio_bias;
  (void)right_audio_bias;

  // ---------------------------------------------------------------------------
  // 6. Placeholder section for future LED / costume logic
  // ---------------------------------------------------------------------------
  //
  // Example ideas:
  
  
   if (strong_audio) {
     run_audio_reactive_effect(sound_level_01);
   }
  
   if (current_motion_flags.impact) {
     trigger_impact_flash();
   }
  
   if (current_motion_flags.step_pulse) {
     trigger_step_pulse();
   }
  
   if (current_motion_flags.spinning) {
     trigger_spin_effect();
   }
  
   if (current_audio_flags.beat_like) {
     trigger_music_pulse();
   }
  
   if (current_audio_flags.clap_like) {
     trigger_sharp_flash();
   }
  
   if (current_audio_flags.loud_event) {
     trigger_loud_burst();
   }
  
   if (leaning_left || left_audio_bias) {
     bias_lights_left();
   }
  
   if (leaning_right || right_audio_bias) {
     bias_lights_right();
   }
  
   if (leaning_forward) {
     bias_lights_front();
   }
  
   if (leaning_backward) {
     bias_lights_back();
   }

  delay(20);
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
float map_to_unit(float value, float min_input, float max_input) {
  if (value <= min_input) {
    return 0.0f;
  }

  if (value >= max_input) {
    return 1.0f;
  }

  return (value - min_input) / (max_input - min_input);
}

void print_motion_compact_status(const Orientation &orientation,
                                 const MotionFeatures &features,
                                 const DetectionFlags &flags) {
  
  Serial.print("motion_energy=");
  Serial.print(features.motion_energy, 3);

  Serial.print(" | roll=");
  Serial.print(orientation.roll_deg, 1);

  Serial.print(" | pitch=");
  Serial.print(orientation.pitch_deg, 1);

  Serial.print(" | impact=");
  Serial.print(flags.impact ? "1" : "0");

  Serial.print(" | step=");
  Serial.print(flags.step_pulse ? "1" : "0");

  Serial.print(" | spin=");
  Serial.println(flags.spinning ? "1" : "0");
}

void print_audio_compact_status(const AudioFeatures &features,
                                const AudioFlags &flags) {

  Serial.print("left_env=");
  Serial.print(features.left_envelope, 5);

  Serial.print(" | right_env=");
  Serial.print(features.right_envelope, 5);

  Serial.print(" | mono_env=");
  Serial.print(features.mono_envelope, 5);

  Serial.print(" | onset=");
  Serial.print(features.onset_strength, 5);

  Serial.print(" | stereo_diff=");
  Serial.print(features.stereo_difference, 5);

  Serial.print(" | side=");
  Serial.print(get_dominant_audio_side(features));

  Serial.print(" | clap=");
  Serial.print(flags.clap_like ? "1" : "0");

  Serial.print(" | beat=");
  Serial.print(flags.beat_like ? "1" : "0");

  Serial.print(" | loud=");
  Serial.println(flags.loud_event ? "1" : "0");
}

const char *get_dominant_audio_side(const AudioFeatures &features) {
  const float side_threshold = 0.01f;

  if (features.left_envelope > features.right_envelope + side_threshold) {
    return "left";
  }

  if (features.right_envelope > features.left_envelope + side_threshold) {
    return "right";
  }

  return "center";
}
