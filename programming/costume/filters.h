// filters.h
// Shared filter classes for motion and audio processing

#ifndef FILTERS_H
#define FILTERS_H

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Generic low-pass filter
// -----------------------------------------------------------------------------
class LowPassFilter {
public:
  LowPassFilter(float alpha = 0.1f, float initial_value = 0.0f);

  float update(float x);
  float value() const;
  void reset(float value = 0.0f);

private:
  float _alpha;
  float _state;
  bool _initialized;
};

// -----------------------------------------------------------------------------
// Generic high-pass filter
// -----------------------------------------------------------------------------
class HighPassFilter {
public:
  HighPassFilter(float alpha = 0.9f);

  float update(float x);
  float value() const;
  void reset();

private:
  float _alpha;
  float _y;
  float _x_prev;
  bool _initialized;
};

// -----------------------------------------------------------------------------
// DC block filter
// y[n] = x[n] - x[n-1] + r * y[n-1]
// Useful for audio
// -----------------------------------------------------------------------------
class DcBlockFilter {
public:
  DcBlockFilter(float r = 0.995f);

  float update(float x);
  float value() const;
  void reset();

private:
  float _r;
  float _x_prev;
  float _y_prev;
  bool _initialized;
};

#endif // FILTERS_H
