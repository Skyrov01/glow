#include <Adafruit_NeoPixel.h>

static const uint8_t led_pin = 27;
static const uint16_t led_count = 10;

Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(32);
  strip.show();
}

void loop() {
  for (uint16_t i = 0; i < led_count; i++) {
    strip.clear();
    strip.setPixelColor(i, strip.Color(0, 0, 255));
    strip.show();
    delay(100);
  }
}
