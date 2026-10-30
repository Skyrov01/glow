
// costume_lights.h
// NeoPixel helper module for the festival costume

#ifndef COSTUME_LIGHTS_H
#define COSTUME_LIGHTS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
struct CostumeLightsConfig {
  uint8_t led_pin;
  uint16_t led_count;
  neoPixelType pixel_type;

  CostumeLightsConfig()
    : led_pin(27),
      led_count(60),
      pixel_type(NEO_GRB + NEO_KHZ800) {}
};

// -----------------------------------------------------------------------------
// Color
// -----------------------------------------------------------------------------
struct CostumeColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;

  CostumeColor()
    : red(0), green(0), blue(0) {}

  CostumeColor(uint8_t r, uint8_t g, uint8_t b)
    : red(r), green(g), blue(b) {}
};

namespace CostumeColors {
  static const CostumeColor black       = CostumeColor(0, 0, 0);
  static const CostumeColor white       = CostumeColor(255, 255, 255);
  static const CostumeColor red         = CostumeColor(255, 0, 0);
  static const CostumeColor green       = CostumeColor(0, 255, 0);
  static const CostumeColor blue        = CostumeColor(0, 0, 255);

  static const CostumeColor yellow      = CostumeColor(255, 180, 0);
  static const CostumeColor orange      = CostumeColor(255, 80, 0);
  static const CostumeColor cyan        = CostumeColor(0, 255, 255);
  static const CostumeColor magenta     = CostumeColor(255, 0, 255);
  static const CostumeColor purple      = CostumeColor(120, 0, 255);
  static const CostumeColor pink        = CostumeColor(255, 20, 120);

  static const CostumeColor warm_white  = CostumeColor(255, 180, 120);
  static const CostumeColor cold_white  = CostumeColor(180, 220, 255);

  static const CostumeColor idle_blue   = CostumeColor(8, 18, 50);
  static const CostumeColor beat_orange = CostumeColor(255, 40, 0);
  static const CostumeColor spin_cyan   = CostumeColor(0, 255, 255);
  static const CostumeColor impact      = CostumeColor(255, 255, 255);
}


static const CostumeColor color_list[] = {
  CostumeColors::black,
  CostumeColors::white,
  CostumeColors::red,
  CostumeColors::green,
  CostumeColors::blue,
  CostumeColors::yellow,
  CostumeColors::orange,
  CostumeColors::cyan,
  CostumeColors::magenta,
  CostumeColors::purple,
  CostumeColors::pink,
  CostumeColors::warm_white,
  CostumeColors::cold_white,
};

static const uint8_t color_list_count = sizeof(color_list) / sizeof(color_list[0]);

// -----------------------------------------------------------------------------
// Spatial configuration
// -----------------------------------------------------------------------------
enum CostumeElementId {
  ELEMENT_NONE = 0,
  ELEMENT_ALL,

  ELEMENT_HEAD,
  ELEMENT_CHEST,
  ELEMENT_BACK,
  ELEMENT_WAIST,

  ELEMENT_LEFT_ARM,
  ELEMENT_RIGHT_ARM,

  ELEMENT_LEFT_FOREARM,
  ELEMENT_RIGHT_FOREARM,

  ELEMENT_LEFT_HAND,
  ELEMENT_RIGHT_HAND,

  ELEMENT_LEFT_LEG,
  ELEMENT_RIGHT_LEG,

  ELEMENT_LEFT_SHIN,
  ELEMENT_RIGHT_SHIN,

  ELEMENT_LEFT_FOOT,
  ELEMENT_RIGHT_FOOT,

  ELEMENT_COUNT
};

struct CostumeElement {
  CostumeElementId id;
  uint16_t start_index;
  uint16_t led_count;

  CostumeElement()
    : id(ELEMENT_NONE),
      start_index(0),
      led_count(0) {}

  CostumeElement(CostumeElementId element_id,
                 uint16_t element_start_index,
                 uint16_t element_led_count)
    : id(element_id),
      start_index(element_start_index),
      led_count(element_led_count) {}
};

// -----------------------------------------------------------------------------
// Main class
// -----------------------------------------------------------------------------
class CostumeLights {
public:
  explicit CostumeLights(const CostumeLightsConfig &config);

  void begin();
  void show();
  void clear();
  void clear_element(CostumeElementId element_id);
  void set_brightness(uint8_t brightness);

  uint16_t get_led_count() const;

  // ---------------------------------------------------------------------------
  // Spatial element configuration
  // ---------------------------------------------------------------------------
  bool set_element(CostumeElementId element_id, uint16_t start_index, uint16_t led_count);
  bool has_element(CostumeElementId element_id) const;
  CostumeElement get_element(CostumeElementId element_id) const;

  uint16_t get_element_start(CostumeElementId element_id) const;
  uint16_t get_element_led_count(CostumeElementId element_id) const;
  uint16_t get_element_end(CostumeElementId element_id) const;

  // ---------------------------------------------------------------------------
  // Raw pixel control
  // ---------------------------------------------------------------------------
  void set_pixel(uint16_t index, const CostumeColor &color_value);

  void fill(const CostumeColor &color_value);


  // ---------------------------------------------------------------------------
  // Element-based control
  // ---------------------------------------------------------------------------
  void set_element_pixel(CostumeElementId element_id, uint16_t local_index, const CostumeColor &color_value);

  void fill_element(CostumeElementId element_id, const CostumeColor &color_value);

  void fill_elements(CostumeElementId first_element_id,
                     CostumeElementId second_element_id,
                     const CostumeColor &color_value);

  void show_element_solid(CostumeElementId element_id, const CostumeColor &color_value);

  void show_element_bar(CostumeElementId element_id, float level_01, const CostumeColor &color_value);

  void show_element_pulse(CostumeElementId element_id, float level_01, const CostumeColor &color_value);

  void show_element_chase(CostumeElementId element_id,
                          uint16_t step,
                          const CostumeColor &color_value,
                          const CostumeColor &background_color = CostumeColors::black);

  void show_element_pair_split(CostumeElementId left_element_id,
                               CostumeElementId right_element_id,
                               float left_level_01,
                               float right_level_01,
                               const CostumeColor &left_color,
                               const CostumeColor &right_color);

private:
  static const uint8_t max_costume_elements = ELEMENT_COUNT;

  CostumeLightsConfig _config;
  Adafruit_NeoPixel _strip;
  CostumeElement _elements[max_costume_elements];

  float clamp_01(float value) const;
  uint16_t level_to_led_count(float level_01) const;
  uint16_t element_level_to_led_count(const CostumeElement &element, float level_01) const;
  uint32_t to_color(const CostumeColor &color_value) const;
  CostumeColor scale_color(const CostumeColor &color_value, float scale_01) const;

  bool is_valid_global_index(uint16_t index) const;
  bool is_valid_element_id(CostumeElementId element_id) const;
};

#endif // COSTUME_LIGHTS_H
