/**
 * @file      AlertFinder.ino
 * @author    Peter Thompson
 * @brief     Main application file for the Alert Finder project. This file
 * handles hardware initialization, Wi-Fi connectivity, and the main
 * program loop. It coordinates the GPS, AlertRetriever, and AlertRenderer modules.
 * @version   3.35 (GPS Debugging)
 * @date      2025-07-30
 *
 * @copyright Copyright (c) 2025
 *
 */

// --- Core Libraries ---
#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>

// --- Hardware & Driver Libraries ---
#include "pin_config.h"
#include "lv_conf.h"
#include "Arduino_GFX_Library.h"
#include "TouchDrvCSTXXX.hpp"
#include "HWCDC.h"
#include <esp_heap_caps.h>

// --- Networking Libraries ---
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// --- Custom Application Modules ---
#include "AlertRenderer.h"
#include "AlertRetriever.h"
#include "GPS.h"
#include "Compass.h"

// --- Wi-Fi Configuration ---
const char* ssid = "TelstraE6FB15";
const char* password = "thebigtent";

// --- Mode Configuration ---
const bool DEMO_MODE = false; // Set to true to run UI demo, false for live data

// --- Global Constants ---
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// --- Global Hardware & Display Objects ---
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0, false, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);

TouchDrvCSTXXX touch;
uint32_t screenWidth;
uint32_t screenHeight;

// --- LVGL Callbacks ---
#if LV_USE_LOG != 0
void my_print(const char *buf) {
  USBSerial.printf(buf);
  USBSerial.flush();
}
#endif

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

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint8_t touched = touch.getPoint(touch_x, touch_y, touch.getSupportTouchPoint());
  if (touched > 0) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touch_x[0];
    data->point.y = touch_y[0];
    if (orientation == 1) {
      data->point.x = screenWidth - 1 - data->point.x;
      data->point.y = screenHeight - 1 - data->point.y;
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/**
 * @brief Strong implementation of the network GET function for the AlertRetriever.
 * This function uses the ESP32's WiFi and HTTPClient libraries.
 * @param host The server host name.
 * @param path The API path.
 * @param payload A reference to a String to store the response body.
 * @return True on success, false on failure.
 */
bool perform_http_get(const char* host, const char* path, String& payload) {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    WiFiClientSecure client;
    HTTPClient http;

    // The Waze API may not have a perfect certificate, so we connect insecurely.
    client.setInsecure();
    http.setTimeout(10000); // 10 second timeout

    String url = "https://" + String(host) + path;

    if (!http.begin(client, url)) {
        USBSerial.println("HTTP GET: Failed to begin connection.");
        http.end();
        return false;
    }

    int httpCode = http.GET();
    if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK) {
            payload = http.getString();
            http.end();
            return true;
        }
    }

    USBSerial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
}

void setup() {
  USBSerial.begin(115200);
  USBSerial.println("AlertFinder starting...");

  // --- STAGE 1: CONNECT TO NETWORK ---
  if (!DEMO_MODE) {
    USBSerial.printf("Connecting to %s ", ssid);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      USBSerial.print(".");
    }
    USBSerial.println("\nWiFi connected!");
    USBSerial.print("IP address: ");
    USBSerial.println(WiFi.localIP());
  } else {
    USBSerial.println("DEMO_MODE is active. Skipping Wi-Fi connection.");
  }

  // --- STAGE 2: INITIALIZE HARDWARE & GRAPHICS ---
  setCpuFrequencyMhz(240);
  Wire.begin(IIC_SDA, IIC_SCL); // Main I2C for Touch, QMI8658, HMC5883L
  touch.setPins(-1, 11);
  if (!touch.begin(Wire, 0x5A, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to initialize touch controller!");
  }
  touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  touch.setMirrorXY(true, true);
  gfx->begin();
  gfx->Display_Brightness(200);
  screenWidth = gfx->width();
  screenHeight = gfx->height();

  renderer_init_lvgl();
  create_arc_gui();
  USBSerial.println("Display and LVGL initialized.");

  // --- STAGE 3: INITIALIZE OTHER MODULES & TIMERS ---
  gps_setup();
  USBSerial.println("GPS module setup complete.");

  if (!compass_setup()) {
    USBSerial.println("Compass module initialization failed!");
  } else {
    USBSerial.println("Compass module setup complete.");
  }

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick, .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  if (!DEMO_MODE) {
    retriever_setup();
  }

  USBSerial.println("Setup done. System is running.");
}

void loop() {
  if (DEMO_MODE) {
    // --- Demo Mode Logic: Cycle through predefined UI states ---
    static uint32_t last_demo_update = 0;
    static int demo_state = 0;

    if (millis() - last_demo_update > 2000) { // Update every 2 seconds
      last_demo_update = millis();
      demo_state++;

      switch (demo_state) {
        case 1:
          USBSerial.println("Demo State 1: No alerts");
          updateAlerts(-1, -1, -1, -1, -1);
          setHeat(0);
          updateAngle(179); // A non-zero angle to test arrow
          updateDistance(-1);
          break;
        case 2:
          USBSerial.println("Demo State 2: 1 Alert");
          updateAlerts(0, -1, -1, -1, -1);
          setHeat(1);
          updateAngle(-90);
          updateDistance(1111);
          break;
        case 3:
          USBSerial.println("Demo State 3: 2 Alerts");
          updateAlerts(0, 0, -1, -1, -1);
          setHeat(2);
          updateAngle(-30);
          updateDistance(930);
          break;
        case 4:
          USBSerial.println("Demo State 4: 3 Alerts");
          updateAlerts(0, 0, 0, -1, -1);
          setHeat(3);
          updateAngle(0);
          updateDistance(400);
          break;
        case 5:
          USBSerial.println("Demo State 5: 4 Alerts");
          updateAlerts(0, 0, 0, 0, -1);
          setHeat(4);
          updateAngle(45);
          updateDistance(300);
          break;
        case 6:
          USBSerial.println("Demo State 6: 5 Alerts");
          updateAlerts(0, 0, 0, 0, 0);
          setHeat(5);
          updateAngle(75);
          updateDistance(150);
          demo_state = 0; // Reset demo cycle
          break;
      }
    }
  } else {
    // --- Live Mode Logic ---
    
    // 1. Update sensor data
    gps_loop();
    compass_loop();

    // --- SENSOR DEBUGGING ---
    static unsigned long last_debug_print = 0;
    if (millis() - last_debug_print > 2000) {
        GPSData gps_data = get_gps_data();
        if (gps_data.is_valid) {
            USBSerial.printf("GPS FIX: Lat=%.6f, Lon=%.6f | ", gps_data.latitude, gps_data.longitude);
        } else {
            USBSerial.print("GPS: No fix | ");
        }
        USBSerial.printf("Compass Heading: %.2f | Orientation: %d\n", get_compass_heading(), get_orientation());
        last_debug_print = millis();
    }

    // 2. Update Module Inputs with latest data
    GPSData gps_data = get_gps_data();
    if (gps_data.is_valid) {
        set_current_location(gps_data.latitude, gps_data.longitude);
    }
    set_current_direction(get_compass_heading());
    set_network_status(WiFi.status() == WL_CONNECTED);
    setOrientation(get_orientation());

    // 3. Run Retriever Loop
    retriever_loop();

    // 4. Check for New Data from Retriever and Update UI
    DisplayData data = get_display_data();
    if (data.data_is_new) {
      USBSerial.printf("New data: H:%d, A:%.1f, D:%d\n", data.heat_level, data.target_angle, data.target_distance_m);
      updateAlerts(data.alert_indices[0], data.alert_indices[1], data.alert_indices[2], data.alert_indices[3], data.alert_indices[4]);
      setHeat(data.heat_level);
      updateAngle(data.target_angle);
      updateDistance(data.target_distance_m);
    }
  }

  // --- Handle LVGL Timer (runs in both modes) ---
  lv_timer_handler();
  delay(5);
}
