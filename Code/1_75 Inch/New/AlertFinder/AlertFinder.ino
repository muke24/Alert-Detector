/**
 * @file      AlertFinder.ino
 * @author    Peter Thompson
 * @brief     Main application file for the Alert Finder project. This file
 * handles hardware initialization and the main program loop. It
 * delegates all UI rendering and management to the AlertRenderer module.
 * @version   3.00 (Refactored)
 * @date      2025-07-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <lvgl.h>
#include <Arduino.h>
#include <Wire.h>
#include "pin_config.h"          // Contains all hardware pin definitions
#include "lv_conf.h"             // LVGL configuration file
#include "Arduino_GFX_Library.h" // Graphics driver
#include "TouchDrvCSTXXX.hpp"    // Touch controller driver
#include "HWCDC.h"               // For USB Serial communication
#include <esp_heap_caps.h>
#include "AlertRenderer.h"       // Include the header for our UI renderer

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// --- Global Hardware & Display Objects ---
// These are defined here and shared with the renderer via `extern` in AlertRenderer.h
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus,
  LCD_RESET /* RST */,
  0 /* rotation */,
  false /* IPS */,
  LCD_WIDTH,
  LCD_HEIGHT,
  6 /*_phases col_offset1 */,
  0 /* row_offset1 */,
  0 /* col_offset2 */,
  0 /* row_offset2 */
);

TouchDrvCSTXXX touch;
uint32_t screenWidth;
uint32_t screenHeight;

/* LVGL log function */
#if LV_USE_LOG != 0
void my_print(const char *buf) {
  USBSerial.printf(buf);
  USBSerial.flush();
}
#endif

/* LVGL display flush callback */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

/* LVGL touch input callback */
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint8_t touched = touch.getPoint(touch_x, touch_y, touch.getSupportTouchPoint());

  if (touched > 0) {
    data->state = LV_INDEV_STATE_PR;  // Set to pressed state
    int16_t tx = touch_x[0];          // Use the first touch point's coordinates
    int16_t ty = touch_y[0];
    // If orientation is flipped, we must manually transform touch coordinates
    // because the display content is manually rotated, not the hardware buffer.
    if (orientation == 1) {
      tx = screenWidth - 1 - tx;
      ty = screenHeight - 1 - ty;
    }
    data->point.x = tx;
    data->point.y = ty;
  } else {
    data->state = LV_INDEV_STATE_REL;  // Set to released state
  }
}

/* Function to increment LVGL's internal tick counter */
void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void setup() {
  USBSerial.begin(115200);

  // Set CPU frequency to maximum for better performance
  setCpuFrequencyMhz(240);

  // Initialize I2C for the touch controller
  Wire.begin(IIC_SDA, IIC_SCL);

  // Initialize Touch Controller with correct pins
  touch.setPins(-1, 11);
  if (!touch.begin(Wire, 0x5A, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to initialize touch controller!");
  }
  touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  touch.setMirrorXY(true, true);

  // Initialize Display
  gfx->begin();
  gfx->Display_Brightness(200);  // Set brightness (0-255)
  screenWidth = gfx->width();
  screenHeight = gfx->height();

  // Initialize LVGL and its drivers using the renderer's setup function
  renderer_init_lvgl();

  // Setup a timer to periodically call lv_tick_inc
  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  // --- Create the custom GUI using the renderer ---
  create_arc_gui();

  // --- Set initial UI state using renderer functions ---
  setOrientation(0);
  setHeat(4);
  updateDistance(666);
  updateAngle(0);
  updateAlerts(0, 0, 0, 0, 0);

  USBSerial.println("Setup done. Arc GUI is running.");
}

void loop() {
  // --- Demo Logic ---
  // This section demonstrates how to control the UI using the public functions
  // from the AlertRenderer module.
  static uint32_t last_update = 0;
  static int demo_state = 0;

  if (millis() - last_update > 2000) { // Update every 2 seconds
    last_update = millis();
    demo_state++;

    switch (demo_state) {
      case 1:
        updateAlerts(-1, -1, -1, -1, -1);
        updateAngle(179);
        updateDistance(-1);
        setHeat(0);
        break;
      case 2:
        updateAlerts(0, -1, -1, -1, -1);
        updateAngle(-90);
        updateDistance(1111);
        setHeat(1);
        break;
      case 3:
        updateAlerts(0, 0, -1, -1, -1);
        updateAngle(-30);
        updateDistance(930);
        setHeat(2);
        break;
      case 4:
        updateAlerts(0, 0, 0, -1, -1);
        updateAngle(0);
        updateDistance(400);
        setHeat(3);
        break;
      case 5:
        updateAlerts(0, 0, 0, 0, -1);
        updateAngle(45);
        updateDistance(300);
        setHeat(4);
        break;
      case 6:
        updateAlerts(0, 0, 0, 0, 0);
        updateAngle(75);
        updateDistance(150);
        setHeat(5);
        demo_state = 0; // Reset demo cycle
        break;
    }
  }

  lv_timer_handler(); // Let the GUI do its work
  delay(1);           // Minimal yield; adjust up if CPU overheats
}
