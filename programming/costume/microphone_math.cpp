// microphone_math.cpp
// Stereo I2S microphone module for two ICS-43434 microphones on ESP32 Feather V2

#include "microphone_math.h"



// -----------------------------------------------------------------------------
// i2s setup / reading
// -----------------------------------------------------------------------------
bool setup_microphones(const I2SMicPins &pins, i2s_port_t i2s_port) {
  i2s_config_t i2s_config = {};
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  i2s_config.sample_rate = 44100;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 64;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = false;
  i2s_config.fixed_mclk = 0;

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = pins.bclk_pin;
  pin_config.ws_io_num = pins.ws_pin;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = pins.data_pin;

  esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.println("Failed to install I2S driver.");
    return false;
  }

  err = i2s_set_pin(i2s_port, &pin_config);
  if (err != ESP_OK) {
    Serial.println("Failed to set I2S pins.");
    return false;
  }

  err = i2s_zero_dma_buffer(i2s_port);
  if (err != ESP_OK) {
    Serial.println("Failed to clear I2S DMA buffer.");
    return false;
  }

  return true;
}

bool read_audio_frame(AudioFrame &frame,
                      unsigned long &last_read_ms,
                      i2s_port_t i2s_port) {
  int32_t sample_buffer[2] = {0, 0};
  size_t bytes_read = 0;

  esp_err_t err = i2s_read(i2s_port, sample_buffer, sizeof(sample_buffer), &bytes_read, portMAX_DELAY);
  if (err != ESP_OK || bytes_read != sizeof(sample_buffer)) {
    return false;
  }

  unsigned long now = millis();
  float dt = (last_read_ms == 0) ? 0.001f : (now - last_read_ms) * 0.001f;
  last_read_ms = now;

  frame.left_sample = sample_buffer[0];
  frame.right_sample = sample_buffer[1];
  frame.t_millis = now;
  frame.dt = dt;

  return true;
}

// -----------------------------------------------------------------------------
// sample math
// -----------------------------------------------------------------------------
float normalize_sample(int32_t sample) {
  return (float)sample / 2147483648.0f;
}

float compute_abs_sample(float sample) {
  return fabsf(sample);
}

float compute_stereo_difference(float left_value, float right_value) {
  return fabsf(left_value - right_value);
}

// -----------------------------------------------------------------------------
// feature extraction
// -----------------------------------------------------------------------------
AudioFeatures compute_audio_features(const AudioFrame &frame, AudioDetectorState &state) {
  AudioFeatures features = {};

  float left_raw = normalize_sample(frame.left_sample);
  float right_raw = normalize_sample(frame.right_sample);

  // TODO(calibration): Microphone gain balance.
  // The two ICS-43434 units may have slightly different sensitivities.
  // Measure the RMS of each channel over ~5 seconds of ambient noise and
  // compute gain_right = rms_left / rms_right (or vice versa). Multiply
  // right_raw by that factor here so stereo_difference reflects true
  // directionality rather than a hardware mismatch.

  float left_filtered = state.left_dc_block.update(left_raw);
  float right_filtered = state.right_dc_block.update(right_raw);

  float left_abs = compute_abs_sample(left_filtered);
  float right_abs = compute_abs_sample(right_filtered);
  float mono_abs = 0.5f * (left_abs + right_abs);

  features.left_peak = left_abs;
  features.right_peak = right_abs;
  features.mono_peak = mono_abs;

  features.left_rms = left_abs;
  features.right_rms = right_abs;
  features.mono_rms = mono_abs;

  features.left_envelope = state.left_envelope_lpf.update(left_abs);
  features.right_envelope = state.right_envelope_lpf.update(right_abs);
  features.mono_envelope = state.mono_envelope_lpf.update(mono_abs);

  features.stereo_difference = compute_stereo_difference(
    features.left_envelope,
    features.right_envelope
  );

  float instantaneous_minus_envelope = mono_abs - features.mono_envelope;
  if (instantaneous_minus_envelope < 0.0f) {
    instantaneous_minus_envelope = 0.0f;
  }
  features.onset_strength = state.onset_lpf.update(instantaneous_minus_envelope);

  return features;
}

// -----------------------------------------------------------------------------
// event detection
// -----------------------------------------------------------------------------
AudioFlags detect_audio_events(const AudioFeatures &features, AudioDetectorState &state) {
  AudioFlags flags = {};
  unsigned long now = millis();

  flags.clap_like = detect_clap_like(
    features.onset_strength,
    features.mono_peak,
    now,
    state.last_clap_ms,
    0.030f,
    0.060f,
    120
  );

  flags.beat_like = detect_beat_like(
    features.mono_envelope,
    now,
    state.last_beat_ms,
    0.020f,
    180,
    900
  );

  flags.loud_event = detect_loud_event(
    features.mono_rms,
    now,
    state.last_loud_event_ms,
    0.10f,
    150
  );

  return flags;
}

bool detect_clap_like(float onset_strength,
                      float mono_peak,
                      unsigned long now_ms,
                      unsigned long &last_trigger_ms,
                      float onset_threshold,
                      float peak_threshold,
                      unsigned long cooldown_ms) {
  if (onset_strength > onset_threshold &&
      mono_peak > peak_threshold &&
      (now_ms - last_trigger_ms) > cooldown_ms) {
    last_trigger_ms = now_ms;
    return true;
  }
  return false;
}

bool detect_beat_like(float mono_envelope,
                      unsigned long now_ms,
                      unsigned long &last_trigger_ms,
                      float threshold,
                      unsigned long min_interval_ms,
                      unsigned long max_interval_ms) {
  if (mono_envelope > threshold) {
    unsigned long dt = now_ms - last_trigger_ms;

    if (dt > min_interval_ms && dt < max_interval_ms) {
      last_trigger_ms = now_ms;
      return true;
    }

    if (last_trigger_ms == 0 || dt >= max_interval_ms) {
      last_trigger_ms = now_ms;
      return true;
    }
  }

  return false;
}

bool detect_loud_event(float mono_rms,
                       unsigned long now_ms,
                       unsigned long &last_trigger_ms,
                       float threshold,
                       unsigned long cooldown_ms) {
  if (mono_rms > threshold && (now_ms - last_trigger_ms) > cooldown_ms) {
    last_trigger_ms = now_ms;
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// debug
// -----------------------------------------------------------------------------
void print_audio_frame(const AudioFrame &frame,
                       const AudioFeatures &features,
                       const AudioFlags &flags) {
  Serial.println("--------------------------------------------------");

  Serial.print("Left sample: ");
  Serial.println(frame.left_sample);

  Serial.print("Right sample: ");
  Serial.println(frame.right_sample);

  Serial.print("Left env: ");
  Serial.println(features.left_envelope, 5);

  Serial.print("Right env: ");
  Serial.println(features.right_envelope, 5);

  Serial.print("Mono env: ");
  Serial.println(features.mono_envelope, 5);

  Serial.print("Onset: ");
  Serial.println(features.onset_strength, 5);

  Serial.print("Stereo diff: ");
  Serial.println(features.stereo_difference, 5);

  Serial.print("Clap: ");
  Serial.println(flags.clap_like ? "YES" : "NO");

  Serial.print("Beat: ");
  Serial.println(flags.beat_like ? "YES" : "NO");

  Serial.print("Loud: ");
  Serial.println(flags.loud_event ? "YES" : "NO");

  Serial.println();
}
