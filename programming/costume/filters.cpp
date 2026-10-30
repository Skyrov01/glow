// filters.cpp
// Shared filter implementations for motion and audio processing

#include "filters.h"

// -----------------------------------------------------------------------------
// low_pass_filter
// -----------------------------------------------------------------------------
LowPassFilter::LowPassFilter(float alpha, float initial_value)
  : _alpha(alpha), _state(initial_value), _initialized(false) {}

float LowPassFilter::update(float x) {
  if (!_initialized) {
    _state = x;
    _initialized = true;
  } else {
    _state = _alpha * x + (1.0f - _alpha) * _state;
  }
  return _state;
}

float LowPassFilter::value() const {
  return _state;
}

void LowPassFilter::reset(float value) {
  _state = value;
  _initialized = false;
}

// -----------------------------------------------------------------------------
// high_pass_filter
// -----------------------------------------------------------------------------
HighPassFilter::HighPassFilter(float alpha)
  : _alpha(alpha), _y(0.0f), _x_prev(0.0f), _initialized(false) {}

float HighPassFilter::update(float x) {
  if (!_initialized) {
    _x_prev = x;
    _y = 0.0f;
    _initialized = true;
    return _y;
  }

  _y = _alpha * (_y + x - _x_prev);
  _x_prev = x;
  return _y;
}

float HighPassFilter::value() const {
  return _y;
}

void HighPassFilter::reset() {
  _y = 0.0f;
  _x_prev = 0.0f;
  _initialized = false;
}

// -----------------------------------------------------------------------------
// dc_block_filter
// -----------------------------------------------------------------------------
DcBlockFilter::DcBlockFilter(float r)
  : _r(r), _x_prev(0.0f), _y_prev(0.0f), _initialized(false) {}

float DcBlockFilter::update(float x) {
  if (!_initialized) {
    _x_prev = x;
    _y_prev = 0.0f;
    _initialized = true;
    return 0.0f;
  }

  float y = x - _x_prev + _r * _y_prev;
  _x_prev = x;
  _y_prev = y;
  return y;
}

float DcBlockFilter::value() const {
  return _y_prev;
}

void DcBlockFilter::reset() {
  _x_prev = 0.0f;
  _y_prev = 0.0f;
  _initialized = false;
}
