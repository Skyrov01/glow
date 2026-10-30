// costume_lights.cpp
// NeoPixel helper module for the festival costume

#include "costume_lights.h"

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CostumeLights::CostumeLights(const CostumeLightsConfig &config)
  : _config(config),
    _strip(config.led_count, config.led_pin, config.pixel_type) {}

// -----------------------------------------------------------------------------
// Basic control
// -----------------------------------------------------------------------------
void CostumeLights::begin() {
  _strip.begin();
  _strip.show();
}

void CostumeLights::show() {
  _strip.show();
}

void CostumeLights::clear() {
  _strip.clear();
  _strip.show();
}

void CostumeLights::set_brightness(uint8_t brightness) {
  _strip.setBrightness(brightness);
  _strip.show();
}

uint16_t CostumeLights::get_led_count() const {
  return _config.led_count;
}

// -----------------------------------------------------------------------------
// Pixel operations
// -----------------------------------------------------------------------------
void CostumeLights::set_pixel(uint16_t index, const CostumeColor &color_value) {
  if (index >= _config.led_count) {
    return;
  }

  _strip.setPixelColor(index, to_color(color_value));
}

void CostumeLights::fill(const CostumeColor &color_value) {
  uint32_t packed = to_color(color_value);
  for (uint16_t i = 0; i < _config.led_count; i++) {
    _strip.setPixelColor(i, packed);
  }
  _strip.show();
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
uint32_t CostumeLights::to_color(const CostumeColor &color_value) const {
  return _strip.Color(color_value.red, color_value.green, color_value.blue);
}

float CostumeLights::clamp_01(float value) const {
  if (value < 0.0f) {
    return 0.0f;
  }

  if (value > 1.0f) {
    return 1.0f;
  }

  return value;
}

uint16_t CostumeLights::level_to_led_count(float level_01) const {
  float clamped_level = clamp_01(level_01);
  return (uint16_t)(clamped_level * _config.led_count + 0.5f);
}


