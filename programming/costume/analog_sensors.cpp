// analog_sensors.cpp

#include "analog_sensors.h"

// -----------------------------------------------------------------------------
// Setup / reading
// -----------------------------------------------------------------------------
bool setup_analog_sensors(const AnalogSensorPins &pins) {
  pinMode(pins.pulse_pin, INPUT);
  pinMode(pins.light_pin, INPUT);

  analogReadResolution(12); // 0..4095 on ESP32

  return true;
}

bool read_analog_sensors(AnalogSensorFrame &frame,
                         unsigned long &last_read_ms,
                         const AnalogSensorPins &pins) {
  int pulse_raw = analogRead(pins.pulse_pin);
  int light_raw = analogRead(pins.light_pin);

  unsigned long now = millis();
  float dt = (last_read_ms == 0) ? 0.01f : (now - last_read_ms) * 0.001f;
  last_read_ms = now;

  frame.pulse_raw = pulse_raw;
  frame.light_raw = light_raw;

  frame.pulse_norm = normalize_adc_12bit(pulse_raw);
  frame.light_norm = normalize_adc_12bit(light_raw);

  frame.t_millis = now;
  frame.dt = dt;

  return true;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
float normalize_adc_12bit(int raw_value) {
  return clamp_01((float)raw_value / 4095.0f);
}

float clamp_01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

// -----------------------------------------------------------------------------
// Feature extraction
// -----------------------------------------------------------------------------
AnalogSensorFeatures compute_analog_features(const AnalogSensorFrame &frame,
                                             AnalogSensorState &state) {
  AnalogSensorFeatures features = {};

  // Pulse sensor
  features.pulse_filtered = state.pulse_lpf.update(frame.pulse_norm);
  features.pulse_baseline = state.pulse_baseline_lpf.update(features.pulse_filtered);
  features.pulse_ac = features.pulse_filtered - features.pulse_baseline;
  features.pulse_strength = state.pulse_strength_lpf.update(fabsf(features.pulse_ac));

  // Light sensor
  features.light_filtered = state.light_lpf.update(frame.light_norm);
  float light_baseline = state.light_baseline_lpf.update(features.light_filtered);
  features.light_delta = features.light_filtered - light_baseline;
  features.light_change_strength = state.light_change_lpf.update(fabsf(features.light_delta));

  return features;
}

// -----------------------------------------------------------------------------
// Event detection
// -----------------------------------------------------------------------------
AnalogSensorFlags detect_analog_events(const AnalogSensorFeatures &features,
                                       AnalogSensorState &state) {
  AnalogSensorFlags flags = {};
  unsigned long now = millis();

  flags.pulse_peak = detect_pulse_peak(
    features.pulse_ac,
    features.pulse_strength,
    now,
    state.last_pulse_peak_ms,
    0.010f,
    0.008f,
    300
  );

  flags.bright_change = detect_bright_change(
    features.light_delta,
    features.light_change_strength,
    now,
    state.last_bright_change_ms,
    0.030f,
    0.015f,
    150
  );

  flags.dark_change = detect_dark_change(
    features.light_delta,
    features.light_change_strength,
    now,
    state.last_dark_change_ms,
    -0.030f,
    0.015f,
    150
  );

  return flags;
}

bool detect_pulse_peak(float pulse_ac,
                       float pulse_strength,
                       unsigned long now_ms,
                       unsigned long &last_trigger_ms,
                       float ac_threshold,
                       float strength_threshold,
                       unsigned long cooldown_ms) {
  if (pulse_ac > ac_threshold &&
      pulse_strength > strength_threshold &&
      (now_ms - last_trigger_ms) > cooldown_ms) {
    last_trigger_ms = now_ms;
    return true;
  }
  return false;
}

bool detect_bright_change(float light_delta,
                          float light_change_strength,
                          unsigned long now_ms,
                          unsigned long &last_trigger_ms,
                          float delta_threshold,
                          float strength_threshold,
                          unsigned long cooldown_ms) {
  if (light_delta > delta_threshold &&
      light_change_strength > strength_threshold &&
      (now_ms - last_trigger_ms) > cooldown_ms) {
    last_trigger_ms = now_ms;
    return true;
  }
  return false;
}

bool detect_dark_change(float light_delta,
                        float light_change_strength,
                        unsigned long now_ms,
                        unsigned long &last_trigger_ms,
                        float delta_threshold,
                        float strength_threshold,
                        unsigned long cooldown_ms) {
  if (light_delta < delta_threshold &&
      light_change_strength > strength_threshold &&
      (now_ms - last_trigger_ms) > cooldown_ms) {
    last_trigger_ms = now_ms;
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// Debug
// -----------------------------------------------------------------------------
void print_analog_frame(const AnalogSensorFrame &frame,
                        const AnalogSensorFeatures &features,
                        const AnalogSensorFlags &flags) {
  Serial.println("--------------------------------------------------");

  Serial.print("Pulse raw: ");
  Serial.println(frame.pulse_raw);

  Serial.print("Pulse norm: ");
  Serial.println(frame.pulse_norm, 4);

  Serial.print("Pulse filtered: ");
  Serial.println(features.pulse_filtered, 4);

  Serial.print("Pulse baseline: ");
  Serial.println(features.pulse_baseline, 4);

  Serial.print("Pulse AC: ");
  Serial.println(features.pulse_ac, 4);

  Serial.print("Pulse strength: ");
  Serial.println(features.pulse_strength, 4);

  Serial.print("Light raw: ");
  Serial.println(frame.light_raw);

  Serial.print("Light norm: ");
  Serial.println(frame.light_norm, 4);

  Serial.print("Light filtered: ");
  Serial.println(features.light_filtered, 4);

  Serial.print("Light delta: ");
  Serial.println(features.light_delta, 4);

  Serial.print("Light change strength: ");
  Serial.println(features.light_change_strength, 4);

  Serial.print("Pulse peak: ");
  Serial.println(flags.pulse_peak ? "YES" : "NO");

  Serial.print("Bright change: ");
  Serial.println(flags.bright_change ? "YES" : "NO");

  Serial.print("Dark change: ");
  Serial.println(flags.dark_change ? "YES" : "NO");

  Serial.println();
}
