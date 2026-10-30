// microphone_math.h

#ifndef MICROPHONE_MATH_H
#define MICROPHONE_MATH_H

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include "filters.h"

struct I2SMicPins {
  int bclk_pin;
  int ws_pin;
  int data_pin;
};

struct AudioFrame {
  int32_t left_sample;
  int32_t right_sample;
  unsigned long t_millis;
  float dt;
};

struct AudioFeatures {
  float left_rms;
  float right_rms;
  float mono_rms;

  float left_peak;
  float right_peak;
  float mono_peak;

  float left_envelope;
  float right_envelope;
  float mono_envelope;

  float stereo_difference;
  float onset_strength;
};

struct AudioFlags {
  bool clap_like;
  bool beat_like;
  bool loud_event;
};

struct AudioDetectorState {
  DcBlockFilter left_dc_block;
  DcBlockFilter right_dc_block;

  LowPassFilter left_envelope_lpf;
  LowPassFilter right_envelope_lpf;
  LowPassFilter mono_envelope_lpf;
  LowPassFilter onset_lpf;

  unsigned long last_clap_ms;
  unsigned long last_beat_ms;
  unsigned long last_loud_event_ms;

  AudioDetectorState()
    : left_dc_block(0.995f),
      right_dc_block(0.995f),
      left_envelope_lpf(0.08f),
      right_envelope_lpf(0.08f),
      mono_envelope_lpf(0.08f),
      onset_lpf(0.15f),
      last_clap_ms(0),
      last_beat_ms(0),
      last_loud_event_ms(0) {}
};

bool setup_microphones(const I2SMicPins &pins, i2s_port_t i2s_port = I2S_NUM_0);
bool read_audio_frame(AudioFrame &frame,
                      unsigned long &last_read_ms,
                      i2s_port_t i2s_port = I2S_NUM_0);

float normalize_sample(int32_t sample);
float compute_abs_sample(float sample);
float compute_stereo_difference(float left_value, float right_value);

AudioFeatures compute_audio_features(const AudioFrame &frame, AudioDetectorState &state);

AudioFlags detect_audio_events(const AudioFeatures &features, AudioDetectorState &state);

bool detect_clap_like(float onset_strength,
                      float mono_peak,
                      unsigned long now_ms,
                      unsigned long &last_trigger_ms,
                      float onset_threshold,
                      float peak_threshold,
                      unsigned long cooldown_ms);

bool detect_beat_like(float mono_envelope,
                      unsigned long now_ms,
                      unsigned long &last_trigger_ms,
                      float threshold,
                      unsigned long min_interval_ms,
                      unsigned long max_interval_ms);

bool detect_loud_event(float mono_rms,
                       unsigned long now_ms,
                       unsigned long &last_trigger_ms,
                       float threshold,
                       unsigned long cooldown_ms);

void print_audio_frame(const AudioFrame &frame,
                       const AudioFeatures &features,
                       const AudioFlags &flags);

#endif // MICROPHONE_MATH_H
