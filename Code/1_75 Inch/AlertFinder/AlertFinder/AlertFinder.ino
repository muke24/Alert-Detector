/**
 * @file      AlertFinder.ino
 * @author    Peter Thompson
 * @brief     A simple GUI for the Waveshare 1.75" Round AMOLED Touch display,
 * showing three arcs, a rotating arrow, and images positioned along an arc.
 * (LVGL 8.4.0 API).
 * @version   2.17
 * @date      2025-07-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <lvgl.h>
#include <Arduino.h>
#include <Wire.h>
#include "pin_config.h"           // Contains all hardware pin definitions
#include "lv_conf.h"              // LVGL configuration file
#include "Arduino_GFX_Library.h"  // Graphics driver
#include "TouchDrvCSTXXX.hpp"     // Touch controller driver
#include "HWCDC.h"                // For USB Serial communication
#include <esp_heap_caps.h>
#include <math.h>
#include "police.c"               // Include the image data file

// The USBSerial object is defined in the core library, so we don't define it here.
// HWCDC USBSerial; // <-- This line was removed to fix the "multiple definition" linker error.

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// Global LVGL and hardware objects
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

uint32_t screenWidth;
uint32_t screenHeight;

// Touch controller variables
TouchDrvCSTXXX touch;
int16_t touch_x[5], touch_y[5];

// Arrow-related objects
lv_obj_t *arrow_img;      // Object for the arrow
lv_obj_t *distance_label; // Global label for distance text

// Angle variables for smooth interpolation
static float target_angle = 0;
static float current_angle = 0;

// Display hardware interface setup (QSPI)
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

// Display driver setup
Arduino_GFX *gfx = new Arduino_CO5300(
  bus,
  LCD_RESET /* RST */,
  0 /* rotation */,
  false /* IPS */,
  LCD_WIDTH,
  LCD_HEIGHT,
  6 /* col_offset1 */,
  0 /* row_offset1 */,
  0 /* col_offset2 */,
  0 /* row_offset2 */
);

// Global colors for bottom arcs
lv_color_t palette_heat_off;
lv_color_t palette_heat_on;
lv_color_t palette_light_grey;

// Global array for bottom arcs
lv_obj_t *bottom_arcs[5];

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
    data->point.x = touch_x[0];       // Use the first touch point's coordinates
    data->point.y = touch_y[0];
  } else {
    data->state = LV_INDEV_STATE_REL;  // Set to released state
  }
}

/* Function to increment LVGL's internal tick counter */
void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/**
 * @brief Event callback for the main screen. Treats any click as a button press.
 * @param e The event data.
 */
static void screen_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    USBSerial.println("Screen clicked! (Button press action)");
    // You can add any action you want to perform on a "button press" here.
  }
}

/**
 * @brief Timer callback to update the arrow's rotation with linear interpolation.
 * @param timer Pointer to the LVGL timer.
 */
static void arrow_update_timer(lv_timer_t *timer) {
  const float smoothing_factor = 0.5;  // Increased for snappier response (less lag)

  // Increment target angle
  target_angle += 10;  // 1 degree per 10ms
  if (target_angle >= 3600) target_angle -= 3600;

  // Calculate shortest angular difference for clockwise rotation
  float delta = target_angle - current_angle;
  if (delta > 1800) delta -= 3600;  // Take shorter clockwise path
  else if (delta < -1800) delta += 3600;  // Avoid anticlockwise path

  // Linearly interpolate current_angle toward target_angle
  current_angle += delta * smoothing_factor;

  // Ensure current_angle stays within 0 to 3600
  if (current_angle >= 3600) current_angle -= 3600;
  if (current_angle < 0) current_angle += 3600;

  lv_obj_set_user_data(arrow_img, (void *)(intptr_t)(int16_t)current_angle);  // Store current angle
  lv_obj_invalidate(arrow_img);  // Force redraw

  // FPS logging (timer fires, not rendered frames—use on-screen perf monitor for real FPS)
  static uint32_t frame_count = 0;
  static uint32_t last_millis = 0;
  frame_count++;
  if (millis() - last_millis >= 1000) {
    USBSerial.printf("Timer FPS: %d\n", frame_count);
    frame_count = 0;
    last_millis = millis();
  }
}

// Base stem points (quad: left head, right head, base right, base left; counter-clockwise)
// (Bottom half of arrow, scaled to 90% of original size)
static lv_point_t base_stem[4] = {
  { -38, 11 },  // Top left point of arrow stem
  { 36, 11 },   // Top right point of arrow stem
  { 9, 111 },   // Bottom right point of arrow stem
  { -11, 111 }  // Bottom left point of arrow stem
};
// Outline stem points (offset 3 pixels outward from centroid {-1, 62.5})
static lv_point_t outline_stem[4] = {
  { -40, 9 },  // Top left point of arrow stem outline
  { 38, 9 },   // Bottom right point of arrow stem outline
  { 10, 114 },  // Bottom right point of arrow stem outline
  { -12, 114 }  // Bottom left point of arrow stem outline
};
// Base head points (triangle: left head, tip, right head; counter-clockwise)
// (Top half of arrow, scaled to 90% of original size)
static lv_point_t base_head[3] = {
  { -74, 21 },  // Bottom left point of arrow head
  { 0, -119 },  // Tip of arrow head
  { 72, 21 }    // Bottom right point of arrow head
};
// Outline head points (offset 3 pixels outward from centroid {-0.67, -22.33})
static lv_point_t outline_head[3] = {
  { -77, 23 },  // Bottom left point of arrow head outline
  { 0, -122 },  // Tip of arrow head outline
  { 75, 23 }    // Bottom right point of arrow head outline
};

// Custom draw callback (moved outside for compilation)
static void arrow_draw_cb(lv_event_t *e) {
  lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
  lv_obj_t *obj = lv_event_get_target(e);

  // Get current angle from user data (set in timer)
  int16_t angle = (int16_t)(intptr_t)lv_obj_get_user_data(obj);

  // Only draw the arrow if it points to the top half of the screen (270° to 360° or 0° to 90°)
  if (angle < 900 || angle > 2700) {
    int16_t cos_a = lv_trigo_cos(angle / 10);  // LVGL trigo funcs for efficiency
    int16_t sin_a = lv_trigo_sin(angle / 10);
    lv_point_t center = { 233, 233 };  // Center of screen (466/2, 466/2)

    // Pivot point at the bottom (scaled to 90% of {0, 123})
    int32_t pivot_x = 0;
    int32_t pivot_y = 111;  // 123 * 0.9

    // Initialize draw descriptor for outline (black fill, no outline)
    lv_draw_rect_dsc_t outline_dsc;
    lv_draw_rect_dsc_init(&outline_dsc);
    outline_dsc.bg_color = lv_color_black();
    outline_dsc.bg_opa = LV_OPA_COVER;
    outline_dsc.outline_width = 0;  // No outline for the outline polygons

    // Rotate and draw outline stem (4 points)
    lv_point_t rotated_outline_stem[4];
    for (int i = 0; i < 4; i++) {
      int32_t dx = outline_stem[i].x;
      int32_t dy = outline_stem[i].y;
      int32_t dx_ = dx - pivot_x;
      int32_t dy_ = dy - pivot_y;
      rotated_outline_stem[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
      rotated_outline_stem[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
    }
    lv_draw_polygon(draw_ctx, &outline_dsc, rotated_outline_stem, 4);

    // Rotate and draw outline head (3 points)
    lv_point_t rotated_outline_head[3];
    for (int i = 0; i < 3; i++) {
      int32_t dx = outline_head[i].x;
      int32_t dy = outline_head[i].y;
      int32_t dx_ = dx - pivot_x;
      int32_t dy_ = dy - pivot_y;
      rotated_outline_head[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
      rotated_outline_head[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
    }
    lv_draw_polygon(draw_ctx, &outline_dsc, rotated_outline_head, 3);

    // Initialize draw descriptor for main arrow (green fill, no outline)
    lv_draw_rect_dsc_t poly_dsc;
    lv_draw_rect_dsc_init(&poly_dsc);
    poly_dsc.bg_color = lv_color_make(128, 255, 0);  // Hardcoded palette_green
    poly_dsc.bg_opa = LV_OPA_COVER;
    poly_dsc.outline_width = 0;  // No outline for main polygons

    // Rotate and draw main stem (4 points)
    lv_point_t rotated_stem[4];
    for (int i = 0; i < 4; i++) {
      int32_t dx = base_stem[i].x;
      int32_t dy = base_stem[i].y;
      int32_t dx_ = dx - pivot_x;
      int32_t dy_ = dy - pivot_y;
      rotated_stem[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
      rotated_stem[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
    }
    lv_draw_polygon(draw_ctx, &poly_dsc, rotated_stem, 4);

    // Rotate and draw main head (3 points)
    lv_point_t rotated_head[3];
    for (int i = 0; i < 3; i++) {
      int32_t dx = base_head[i].x;
      int32_t dy = base_head[i].y;
      int32_t dx_ = dx - pivot_x;
      int32_t dy_ = dy - pivot_y;
      rotated_head[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
      rotated_head[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
    }
    lv_draw_polygon(draw_ctx, &poly_dsc, rotated_head, 3);
  }
}

/**
 * @brief Sets the heat level by changing the color of the bottom arcs.
 * @param heatLevel The heat level (0 to 5).
 */
void setHeat(int heatLevel) {
  if (heatLevel < 0) heatLevel = 0;
  if (heatLevel > 5) heatLevel = 5;
  for (int i = 0; i < 5; i++) {
    lv_color_t color = (i < heatLevel) ? palette_heat_on : palette_light_grey;
    lv_obj_set_style_arc_color(bottom_arcs[i], color, LV_PART_INDICATOR);
  }
}

/**
 * @brief Updates the distance text based on the input meters, rounded to the nearest 50 meters.
 * @param meters The distance in meters.
 */
void updateDistance(int meters) {
  // Round to nearest 50 meters
  int rounded_meters = round(meters / 50.0) * 50;
  
  // Buffer to hold the formatted string
  char distance_text[16];
  
  if (rounded_meters < 1000) {
    // Format as meters (e.g., "650m")
    snprintf(distance_text, sizeof(distance_text), "%dm", rounded_meters);
  } else {
    // Convert to kilometers and format to 2 decimal places (e.g., "1.25km")
    float kilometers = rounded_meters / 1000.0f;
    snprintf(distance_text, sizeof(distance_text), "%.2fkm", kilometers);
  }
  
  // Update the label text
  lv_label_set_text(distance_label, distance_text);
}

/**
 * @brief Creates the GUI with three concentric semi-circle arcs, a rotating arrow, and images positioned along an arc.
 * Updated to use LVGL v8 styling API and style from reference script.
 */
void create_arc_gui() {
  // Define custom colors from the reference script
  lv_color_t palette_dark_green = lv_color_make(55, 107, 0);
  lv_color_t palette_green = lv_color_make(128, 255, 0);
  palette_light_grey = lv_color_make(30, 30, 30);  // New color for bottom arcs
  palette_heat_off = lv_color_make(20, 20, 20);  // New color for bottom arcs
  palette_heat_on = lv_color_make(255, 106, 0);  // Only for use later when logic is added for bottom arcs

  // Get the active screen object
  lv_obj_t *scr = lv_scr_act();

  // Set a black background for the screen
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  // Add a global event callback to the screen to capture all clicks
  lv_obj_add_event_cb(scr, screen_event_cb, LV_EVENT_ALL, NULL);

  // Background Arcs on bottom half of screen
  // --- Arc 4 (Inner) ---
  lv_obj_t *arc4 = lv_arc_create(scr);
  lv_obj_set_size(arc4, 336, 336);
  lv_obj_align(arc4, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_bg_angles(arc4, 0, 180);
  lv_arc_set_range(arc4, -90, 90);
  lv_arc_set_value(arc4, 90);

  // Style the arc
  lv_obj_remove_style(arc4, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc4, LV_OBJ_FLAG_CLICKABLE);  // Make it non-interactive
  lv_obj_set_style_arc_color(arc4, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc4, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc4, palette_heat_off, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc4, LV_OPA_COVER, LV_PART_INDICATOR);
  // Set width to radius (336 / 2 = 168) to create a solid semi-circle
  lv_obj_set_style_arc_width(arc4, 168, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc4, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc4, 0, LV_PART_MAIN);  // Disable background arc

  // --- Arc 5 (Middle) ---
  lv_obj_t *arc5 = lv_arc_create(scr);
  lv_obj_set_size(arc5, 355, 355);
  lv_obj_align(arc5, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_bg_angles(arc5, 0, 180);
  lv_arc_set_range(arc5, -90, 90);  // Use a 0-180 range to be explicit
  lv_arc_set_value(arc5, 90);     // Set to max to ensure it's a full semi-circle

  // Style the arc
  lv_obj_remove_style(arc5, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc5, LV_OBJ_FLAG_CLICKABLE);  // Make it non-interactive
  lv_obj_set_style_arc_color(arc5, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc5, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc5, palette_light_grey, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc5, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc5, 60, LV_PART_INDICATOR);  // Increased thickness
  lv_obj_set_style_arc_rounded(arc5, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc5, 0, LV_PART_MAIN);  // Disable background arc

  // TODO: 
  // - Make start and end angle change depending on how many images there are (so that an even amount of images are centered).
  // - Fix start and end angle so that they are below the arcs at the top and so the images (police img ect.) do not clip with them
  //
  // --- Images positioned along an arc in the bottom half (created first to be below other UI) ---
  const int num_positions = 5;  // Set amount of positions for images
  float image_arc_r = 150.0f;  // Smaller radius to avoid covering UI elements
  float start_angle = 90.0f;   // Start at 36° to center the 108° arc
  float end_angle = 0.0f;    // End at 144° (108° span, centered in bottom half)
  float spacing = (end_angle - start_angle) / (num_positions + 1);  // Tighter spacing
  int center_x = 233;
  int center_y = 233;
  int img_size = 80;  // Image size 80x80

  for (int i = 0; i < num_positions; i++) {
    float angle = start_angle + (i + 1) * spacing;  // Shift towards middle
    // Calculate position for the center of the image
    int32_t img_x = center_x + (int32_t)roundf(image_arc_r * lv_trigo_cos((int32_t)roundf(angle) * 10) / LV_TRIGO_SIN_MAX);
    int32_t img_y = center_y + (int32_t)roundf(image_arc_r * lv_trigo_sin((int32_t)roundf(angle) * 10) / LV_TRIGO_SIN_MAX);

    if (i == 2) {  // Place the police image at the center position (bottom center)
      lv_obj_t *policeImg = lv_img_create(scr);
      lv_img_set_src(policeImg, &police);
      lv_obj_set_pos(policeImg, img_x - (img_size / 2), img_y - (img_size / 2));
      lv_obj_set_size(policeImg, img_size, img_size);
    }
    // To add more images later, duplicate the if block for other i values (0 to 4),
    // create new lv_img_create, set different src, and use the same pos calculation.
  }

  // --- Arc 3 (Inner) ---
  lv_obj_t *arc3 = lv_arc_create(scr);
  lv_obj_set_size(arc3, 336, 336);
  lv_obj_align(arc3, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_bg_angles(arc3, 180, 0);
  lv_arc_set_range(arc3, -90, 90);
  lv_arc_set_value(arc3, 90);

  // Style the arc
  lv_obj_remove_style(arc3, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc3, LV_OBJ_FLAG_CLICKABLE);  // Make it non-interactive
  lv_obj_set_style_arc_color(arc3, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc3, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc3, palette_dark_green, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc3, LV_OPA_COVER, LV_PART_INDICATOR);
  // Set width to radius (336 / 2 = 168) to create a solid semi-circle
  lv_obj_set_style_arc_width(arc3, 168, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc3, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc3, 0, LV_PART_MAIN);  // Disable background arc

  // --- Arc 2 (Middle) ---
  lv_obj_t *arc2 = lv_arc_create(scr);
  lv_obj_set_size(arc2, 355, 355);
  lv_obj_align(arc2, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_bg_angles(arc2, 180, 0);
  lv_arc_set_range(arc2, 0, 180);  // Use a 0-180 range to be explicit
  lv_arc_set_value(arc2, 180);     // Set to max to ensure it's a full semi-circle

  // Style the arc
  lv_obj_remove_style(arc2, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc2, LV_OBJ_FLAG_CLICKABLE);  // Make it non-interactive
  lv_obj_set_style_arc_color(arc2, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc2, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc2, palette_green, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc2, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc2, 60, LV_PART_INDICATOR);  // Increased thickness
  lv_obj_set_style_arc_rounded(arc2, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc2, 0, LV_PART_MAIN);  // Disable background arc

  // --- Arc 1 (Outer) ---
  lv_obj_t *arc1 = lv_arc_create(scr);
  lv_obj_set_size(arc1, 466, 466);
  lv_obj_align(arc1, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_bg_angles(arc1, 180, 0);  // Draws the top half
  lv_arc_set_range(arc1, -90, 90);
  lv_arc_set_value(arc1, 90);  // Set to max to make it a static, full arc

  // Style the arc like AlertRenderer.ino
  lv_obj_remove_style(arc1, NULL, LV_PART_KNOB);                     // Remove draggable knob
  lv_obj_clear_flag(arc1, LV_OBJ_FLAG_CLICKABLE);                    // Make it non-interactive
  lv_obj_set_style_arc_color(arc1, lv_color_black(), LV_PART_MAIN);  // Background of the arc itself
  lv_obj_set_style_arc_opa(arc1, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc1, palette_dark_green, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc1, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc1, 25, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc1, false, LV_PART_INDICATOR);  // Sharp ends
  lv_obj_set_style_arc_width(arc1, 0, LV_PART_MAIN);  // Disable background arc

  // --- Additional 5 arcs on the opposite side (top, facing opposite to Arc 1) ---
  #define PI 3.1415926535f
  float r = 466 / 2.0f;  // Outer radius approximation
  float gap_px = 25.0f;
  float angle_gap = (gap_px / r) * (180.0f / PI);  // In degrees
  int num_arcs = 5;
  float total_gaps = (num_arcs - 1) * angle_gap;
  float total_arcs_angle = 180.0f - total_gaps;
  float each_arc_angle = total_arcs_angle / num_arcs;

  float current_start = 0.0f;  // Start at 0° for the top semi-circle
  for (int i = 0; i < num_arcs; i++) {
    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 466, 466);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);

    float current_end = current_start + each_arc_angle;

    lv_arc_set_bg_angles(arc, (uint16_t)roundf(current_start), (uint16_t)roundf(current_end));
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 100);

    // Style same as Arc 1 but with new orange color
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, palette_light_grey, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 25, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 0, LV_PART_MAIN);  // Disable background arc

    bottom_arcs[i] = arc;

    current_start = current_end + angle_gap;
  }

  // --- Arrow (Drawn directly with rotated polygon) ---
  // Create a container object for the arrow to allow easy styling and positioning
  arrow_img = lv_obj_create(scr);        // Reusing arrow_img variable for the obj
  lv_obj_set_size(arrow_img, 466, 466);  // Bounding box for the arrow (scaled to display size)
  lv_obj_align(arrow_img, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(arrow_img, lv_color_make(0, 0, 0), LV_PART_MAIN);  // Minimal bg to force draw
  lv_obj_set_style_bg_opa(arrow_img, 1, LV_PART_MAIN);                         // Custom minimal opa (1/255) to ensure draw callback triggers without visibility
  lv_obj_set_style_border_opa(arrow_img, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_add_flag(arrow_img, LV_OBJ_FLAG_EVENT_BUBBLE | LV_OBJ_FLAG_OVERFLOW_VISIBLE);  // Allow clicks to pass through and prevent clipping

  lv_obj_add_event_cb(arrow_img, arrow_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
  lv_obj_set_user_data(arrow_img, (void *)(intptr_t)0);  // Initial angle

  // Add text below the arrow
  distance_label = lv_label_create(scr);
  lv_label_set_text(distance_label, "200m");
  lv_obj_set_style_text_color(distance_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(distance_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_align(distance_label, LV_ALIGN_CENTER, 0, 50);
}

void setup() {
  USBSerial.begin(115200);

  // Set CPU frequency to maximum for better performance
  setCpuFrequencyMhz(240);

  // Initialize I2C for the touch controller
  Wire.begin(IIC_SDA, IIC_SCL);

  // Initialize Touch Controller with correct pins
  // From the provided demo code: RST is -1 (not used), INT is 11
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

  // Initialize LVGL
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  // Allocate LVGL display buffers in DMA-capable internal memory
  uint32_t buf_rows = 120;  // ~25% height for better perf (~224KB total, fits in ~297KB free internal heap)
  buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  // Check allocation and fallback if failed
  if (buf1 == NULL || buf2 == NULL) {
    USBSerial.println("Large internal buffer allocation failed! Falling back to minimal.");
    buf_rows = 40;
    if (buf1 == NULL) buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf2 == NULL) buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  }

  // Final check: Halt if still failed (prevents crash)
  if (buf1 == NULL || buf2 == NULL) {
    USBSerial.println("Buffer allocation failed completely! Cannot proceed.");
    while (1);  // Halt to avoid Guru Meditation crash
  }

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * buf_rows);

  // Log heap info for debugging
  USBSerial.printf("Buffer rows: %d, Buf1: %p, Buf2: %p\n", buf_rows, buf1, buf2);
  USBSerial.printf("Free internal heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  USBSerial.printf("Free PSRAM heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  // Initialize LVGL Display Driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Initialize LVGL Input Device Driver (Touch)
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  // Setup a timer to periodically call lv_tick_inc
  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  // --- Create the custom GUI ---
  create_arc_gui();

  // Set initial heat level
  setHeat(4);

  // Set distance text
  updateDistance(566);

  // Create a timer to update the arrow rotation
  lv_timer_create(arrow_update_timer, 10, NULL);  // Update every 10ms for smooth rotation

  USBSerial.println("Setup done. Arc GUI is running.");
}

void loop() {
  lv_timer_handler();  // Let the GUI do its work
  delay(1);            // Minimal yield; adjust up if CPU overheats
}