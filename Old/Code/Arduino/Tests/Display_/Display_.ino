/*
 * Minimal GFX Hardware Test for Waveshare 1.28" GC9A01 Display
 * This sketch does nothing but initialize the display and cycle colors.
 * If this does not work, the problem is wiring or a hardware/library incompatibility.
*/

#include <Arduino_GFX_Library.h>

// Pin definitions MUST match your physical wiring
#define GFX_BL   12
#define GFX_DC   32
#define GFX_CS   19
#define GFX_SCK  18
#define GFX_MOSI 23
#define GFX_RST  33

// GFX objects, identical to your main project
Arduino_DataBus *bus = new Arduino_ESP32SPI(GFX_DC, GFX_CS, GFX_SCK, GFX_MOSI, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_RST, 0 /* rotation */, true /* IPS */);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting Minimal GFX Hardware Test...");

  // Turn on the backlight
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);

  // Initialize the display driver
  if (!gfx->begin()) {
    Serial.println("FATAL: gfx->begin() failed. Check all wiring.");
    while (1); // Halt forever if initialization fails
  }
  Serial.println("GFX initialization successful.");
}

void loop() {
  // Cycle through full-screen colors every second.
  // This continuously sends commands to the display.
  
  Serial.println("Drawing RED");
  gfx->fillScreen(RED);
  delay(1000);

  Serial.println("Drawing GREEN");
  gfx->fillScreen(GREEN);
  delay(1000);

  Serial.println("Drawing BLUE");
  gfx->fillScreen(BLUE);
  delay(1000);
}