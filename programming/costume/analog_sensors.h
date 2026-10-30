// analog_sensors.h
// Analog sensor module for ESP32 wearable costume
// Supports:
//   - Pulse / heart-rate analog sensor
//   - TEMT6000 ambient light sensor

#ifndef ANALOG_SENSORS_H
#define ANALOG_SENSORS_H

#include <Arduino.h>
#include "filters.h"

// -----------------------------------------------------------------------------
// Pin configuration
// -----------------------------------------------------------------------------
struct AnalogSensorPins {
  int pulse_pin;
  int light_pin;
};

// -----------------------------------------------------------------------------
// Raw frame
// -----------------------------------------------------------------------------
struct AnalogSensorFrame {
  int pulse_raw;
  int light_raw;

  float pulse_norm;
  float light_norm;

  unsigned long t_millis;
  float dt;
};

// -----------------------------------------------------------------------------
// Computed features
// -----------------------------------------------------------------------------
struct AnalogSensorFeatures {
  float pulse_filtered;
  float pulse_baseline;
  float pulse_ac;
  float pulse_strength;

  float light_filtered;
  float light_delta;
  float light_change_strength;
};

// -----------------------------------------------------------------------------
// Event flags
// -----------------------------------------------------------------------------
struct AnalogSensorFlags {
  bool pulse_peak;
  bool bright_change;
  bool dark_change;
};

// -----------------------------------------------------------------------------
// Detector / filter state
// -----------------------------------------------------------------------------
struct AnalogSensorState {
  LowPassFilter pulse_lpf;
  LowPassFilter pulse_baseline_lpf;
  LowPassFilter pulse_strength_lpf;

  LowPassFilter light_lpf;
  LowPassFilter light_baseline_lpf;
  LowPassFilter light_change_lpf;

  unsigned long last_pulse_peak_ms;
  unsigned long last_bright_change_ms;
  unsigned long last_dark_change_ms;

  AnalogSensorState()
    : pulse_lpf(0.18f),
      pulse_baseline_lpf(0.02f),
      pulse_strength_lpf(0.20f),
      light_lpf(0.12f),
      light_baseline_lpf(0.03f),
      light_change_lpf(0.15f),
      last_pulse_peak_ms(0),
      last_bright_change_ms(0),
      last_dark_change_ms(0) {}
};

// -----------------------------------------------------------------------------
// Setup / reading
// -----------------------------------------------------------------------------
bool setup_analog_sensors(const AnalogSensorPins &pins);
bool read_analog_sensors(AnalogSensorFrame &frame,
                         unsigned long &last_read_ms,
                         const AnalogSensorPins &pins);

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
float normalize_adc_12bit(int raw_value);
float clamp_01(float value);

// -----------------------------------------------------------------------------
// Feature extraction
// -----------------------------------------------------------------------------
AnalogSensorFeatures compute_analog_features(const AnalogSensorFrame &frame,
                                             AnalogSensorState &state);

// -----------------------------------------------------------------------------
// Event detection
// -----------------------------------------------------------------------------
AnalogSensorFlags detect_analog_events(const AnalogSensorFeatures &features,
                                       AnalogSensorState &state);

bool detect_pulse_peak(float pulse_ac,
                       float pulse_strength,
                       unsigned long now_ms,
                       unsigned long &last_trigger_ms,
                       float ac_threshold,
                       float strength_threshold,
                       unsigned long cooldown_ms);

bool detect_bright_change(float light_delta,
                          float light_change_strength,
                          unsigned long now_ms,
                          unsigned long &last_trigger_ms,
                          float delta_threshold,
                          float strength_threshold,
                          unsigned long cooldown_ms);

bool detect_dark_change(float light_delta,
                        float light_change_strength,
                        unsigned long now_ms,
                        unsigned long &last_trigger_ms,
                        float delta_threshold,
                        float strength_threshold,
                        unsigned long cooldown_ms);

// -----------------------------------------------------------------------------
// Debug
// -----------------------------------------------------------------------------
void print_analog_frame(const AnalogSensorFrame &frame,
                        const AnalogSensorFeatures &features,
                        const AnalogSensorFlags &flags);

#endif // ANALOG_SENSORS_H
