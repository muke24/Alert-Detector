/**
 * @file      AlertFinder.ino
 * @author    Peter Thompson
 * @brief     A simple GUI for the Waveshare 1.75" Round AMOLED Touch display,
 * showing three arcs, a rotating arrow, and images positioned along an arc.
 * This version implements manual UI rotation and explicit arrow angle control.
 * (LVGL 8.4.0 API).
 * @version   2.50 (Refactored)
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
#include "police.c"  // Include the image data file

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define PI 3.1415926535f  // Define PI globally for use in multiple functions

// Global LVGL and hardware objects
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

uint32_t screenWidth;
uint32_t screenHeight;

// Touch controller variables
TouchDrvCSTXXX touch;
int16_t touch_x[5], touch_y[5];

// --- UI Object Handles ---
// These are now global to allow modification for orientation changes.
lv_obj_t *arrow_img;
lv_obj_t *distance_label;
lv_obj_t *arc1, *arc2, *arc3, *arc4, *arc5;
lv_obj_t *bottom_arcs[5];
lv_obj_t *images[5];  // Array to hold image objects

// Angle variables for smooth interpolation
// target_angle is now set externally by updateAngle()
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
  6 /*_phases col_offset1 */,
  0 /* row_offset1 */,
  0 /* col_offset2 */,
  0 /* row_offset2 */
);

// Global colors for bottom arcs
lv_color_t palette_dark_grey;
lv_color_t palette_light_grey;
lv_color_t palette_heat_on;

// Global orientation flag (0: normal, 1: flipped 180 degrees)
int orientation = 0;

// --- Dependencies for updateAlerts ---

// 1. Array mapping integer indices to the actual image data.
//    Add more declared images here as you create them (e.g., &ambulance, &fire_truck).
LV_IMG_DECLARE(police);
const lv_img_dsc_t *alert_image_sources[] = {
  &police
};
const int NUM_ALERT_TYPES = sizeof(alert_image_sources) / sizeof(alert_image_sources[0]);

// 2. Custom animation callback to set object opacity, required for LVGL v8 animations.
void anim_set_opa_cb(void *var, int32_t v) {
  lv_obj_set_style_opa((lv_obj_t *)var, v, 0);
}

// Forward declaration for the function that applies orientation changes
void apply_orientation();

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
 * This timer now smoothly moves the arrow towards the global `target_angle`.
 * @param timer Pointer to the LVGL timer.
 */
static void arrow_update_timer(lv_timer_t *timer) {
  // A smaller factor creates a smoother, slower animation.
  const float smoothing_factor = 0.1f;

  // The target angle is no longer incremented here. It is set by the updateAngle() function.

  // Calculate the shortest angular difference to the target.
  float delta = target_angle - current_angle;

  // Adjust delta to ensure the arrow takes the shortest path (e.g., -30° instead of +330°).
  if (delta > 1800) delta -= 3600;
  else if (delta < -1800) delta += 3600;

  // If we are very close to the target, just snap to it to prevent tiny oscillations.
  if (fabs(delta) < 1.0f) {
    current_angle = target_angle;
  } else {
    // Linearly interpolate current_angle toward target_angle.
    current_angle += delta * smoothing_factor;
  }

  // Ensure current_angle stays within the 0 to 3599 range.
  if (current_angle >= 3600) current_angle -= 3600;
  if (current_angle < 0) current_angle += 3600;

  // Store the new angle in the object's user data and invalidate it to force a redraw.
  lv_obj_set_user_data(arrow_img, (void *)(intptr_t)(int16_t)current_angle);
  lv_obj_invalidate(arrow_img);

  // FPS logging (optional)
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
  { -40, 9 },   // Top left point of arrow stem outline
  { 38, 9 },    // Bottom right point of arrow stem outline
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
  { 0, -122 },  // Tip arrow head outline
  { 75, 23 }    // Bottom right point of arrow head outline
};

/**
 * @brief Custom draw callback for the rotating arrow.
 * It now considers the global `orientation` flag to rotate the drawing by 180 degrees if needed.
 * @param e The event data.
 */
static void arrow_draw_cb(lv_event_t *e) {
  lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
  lv_obj_t *obj = lv_event_get_target(e);

  // Get current angle from user data (set in timer)
  int16_t angle = (int16_t)(intptr_t)lv_obj_get_user_data(obj);

  // Only draw the arrow if it points to the logical top half of the screen (270° to 360° or 0° to 90°)
  if (angle < 900 || angle > 2700) {
    // If orientation is flipped, add 180 degrees to the angle for rendering
    // This makes the arrow appear correctly on the upside-down UI
    if (orientation == 1) {
      angle = (angle + 1800) % 3600;
    }

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

int current_heat_level = 0;

/**
 * @brief Sets the heat level by animating the fill of the bottom arcs like a progress bar.
 * @param heatLevel The heat level (0 to 5).
 */
void setHeat(int heatLevel) {
  if (heatLevel < 0) heatLevel = 0;
  if (heatLevel > 5) heatLevel = 5;

  int prev_heat_level = current_heat_level;
  current_heat_level = heatLevel;

  int delta = abs(heatLevel - prev_heat_level);
  if (delta == 0) return;

  int base_anim_time = 500;  // ms for single arc change
  int anim_time = base_anim_time / delta;

  for (int i = 0; i < 5; i++) {
    int target_value = (i < heatLevel) ? 100 : 0;
    int current_value = lv_arc_get_value(bottom_arcs[i]);
    if (current_value != target_value) {
      // Stop any ongoing value animation on this arc
      lv_anim_del(bottom_arcs[i], (lv_anim_exec_xcb_t)lv_arc_set_value);

      // Calculate delay based on direction
      uint32_t delay = 0;
      if (heatLevel > prev_heat_level) {
        // Increasing: lower arcs first (smaller i first)
        delay = (i - prev_heat_level) * anim_time;
      } else if (heatLevel < prev_heat_level) {
        // Decreasing: higher arcs first (larger i first)
        delay = (prev_heat_level - 1 - i) * anim_time;
      }

      // Create new animation
      lv_anim_t a;
      lv_anim_init(&a);
      lv_anim_set_var(&a, bottom_arcs[i]);
      lv_anim_set_values(&a, current_value, target_value);
      lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_arc_set_value);
      lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
      lv_anim_set_time(&a, anim_time);
      lv_anim_set_delay(&a, delay);
      lv_anim_start(&a);

      // Ensure flat ends are maintained during animation
      lv_obj_set_style_arc_rounded(bottom_arcs[i], false, LV_PART_INDICATOR);
      lv_obj_set_style_arc_rounded(bottom_arcs[i], false, LV_PART_MAIN);
    }
  }
}

/**
 * @brief Updates the distance text based on the input meters, rounded to the nearest 50 meters.
 * @param meters The distance in meters.
 */
void updateDistance(int meters) {
  if (meters != -1) {
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
  } else {
    // Update the label text
    lv_label_set_text(distance_label, "");
  }
}

/**
 * @brief Sets the target angle for the arrow, which will then smoothly animate to the new position.
 * @param angle The target angle in degrees (0-359).
 */
void updateAngle(int angle) {
  // Ensure angle is within a valid 0-359 degree range
  angle = angle % 360;
  if (angle < 0) {
    angle += 360;
  }

  // Convert to tenths of a degree for internal calculations and set the global target.
  // This value will be picked up by the arrow_update_timer.
  target_angle = angle * 10;
}

/**
 * @brief Updates, positions, and animates alert icons on a bottom arc.
 * @param alertIndex0 Index for the 1st icon (-1 for none).
 * @param alertIndex1 Index for the 2nd icon (-1 for none).
 * @param alertIndex2 Index for the 3rd icon (-1 for none).
 * @param alertIndex3 Index for the 4th icon (-1 for none).
 * @param alertIndex4 Index for the 5th icon (-1 for none).
 *
 * Icons appear from the right, fade in, and move to their calculated
 * positions. The final arrangement depends on the number of active alerts.
 */
void updateAlerts(int alertIndex0, int alertIndex1, int alertIndex2, int alertIndex3, int alertIndex4) {
  int input_indices[5] = { alertIndex0, alertIndex1, alertIndex2, alertIndex3, alertIndex4 };
  int active_indices[5];
  int num_active_alerts = 0;

  // Collect all valid (non -1) alert indices, preserving their order
  for (int i = 0; i < 5; i++) {
    if (input_indices[i] != -1) {
      active_indices[num_active_alerts] = input_indices[i];
      num_active_alerts++;
    }
  }

  // First, hide all image objects to clear the previous state.
  // We will only unhide and animate the ones needed for the current call.
  for (int i = 0; i < 5; i++) {
    if (images[i]) {
      lv_anim_del(images[i], NULL);  // Stop any ongoing animations on the object
      lv_obj_add_flag(images[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (num_active_alerts == 0) {
    return;  // No alerts to display
  }

  // --- Layout & Animation Constants ---
  const float image_arc_r = 150.0f;
  const int center_x = screenWidth / 2;
  const int center_y = screenHeight / 2;
  const int img_size = 64;
  const uint32_t anim_duration = 700;  // Total animation time in ms
  const uint32_t anim_stagger = 100;   // Delay between each icon animation in ms

  // Define the angular span (in degrees) of the arc based on the number of active alerts.
  // A wider span is used for more icons, making them spread out.
  const float spans[] = { 0.0f, 0.0f, 40.0f, 70.0f, 100.0f, 120.0f };  // Index 0 is a placeholder
  float total_span_deg = spans[num_active_alerts];

  // Determine the arc's center based on screen orientation (90° = bottom, 270° = top)
  float center_angle_deg = (orientation == 0) ? 90.0f : 270.0f;
  float start_angle_deg = center_angle_deg - total_span_deg / 2.0f;
  float angle_step_deg = (num_active_alerts > 1) ? total_span_deg / (num_active_alerts - 1) : 0;

  // --- Animation Start Position ---
  // Icons will animate from the right-hand midpoint of the circular path (0 degrees).
  float anim_start_rad = 0.0f;
  int32_t anim_start_x = center_x + (int32_t)(image_arc_r * cosf(anim_start_rad)) - (img_size / 2);
  int32_t anim_start_y = center_y + (int32_t)(image_arc_r * sinf(anim_start_rad)) - (img_size / 2);

  // Loop through the active alerts to configure and start their animations
  for (int i = 0; i < num_active_alerts; i++) {
    int alert_type_index = active_indices[i];
    lv_obj_t *img_obj = images[i];  // Reuse an image object from our global pool

    // Skip if the image object doesn't exist or the index is invalid
    if (!img_obj || alert_type_index < 0 || alert_type_index >= NUM_ALERT_TYPES) {
      continue;
    }

    // 1. Configure the image object for animation
    lv_obj_set_size(img_obj, img_size, img_size);
    lv_img_set_src(img_obj, alert_image_sources[alert_type_index]);
    lv_obj_set_pos(img_obj, anim_start_x, anim_start_y);
    lv_obj_set_style_opa(img_obj, LV_OPA_TRANSP, 0);  // Start fully transparent
    lv_obj_clear_flag(img_obj, LV_OBJ_FLAG_HIDDEN);   // Make it visible

    // 2. Calculate the final target position on the arc
    float target_angle_deg = start_angle_deg + i * angle_step_deg;
    float target_angle_rad = target_angle_deg * (PI / 180.0f);
    int32_t target_x = center_x + (int32_t)(image_arc_r * cosf(target_angle_rad)) - (img_size / 2);
    int32_t target_y = center_y + (int32_t)(image_arc_r * sinf(target_angle_rad)) - (img_size / 2);

    // 3. Initialize the animation template
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, img_obj);
    lv_anim_set_time(&a, anim_duration);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_delay(&a, i * anim_stagger);

    // --- Create and start the animations for X, Y, and Opacity ---

    // Animate X position
    lv_anim_set_values(&a, anim_start_x, target_x);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_start(&a);

    // Animate Y position
    lv_anim_set_values(&a, anim_start_y, target_y);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_start(&a);

    // Animate Opacity (fade-in)
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_exec_cb(&a, anim_set_opa_cb);
    lv_anim_set_time(&a, anim_duration / 2);  // Fade in happens in the first half of the move
    lv_anim_start(&a);

    // Apply rotation based on orientation to keep the image upright
    int16_t rotation_angle = (orientation == 1) ? 1800 : 0;
    lv_obj_set_style_transform_angle(img_obj, rotation_angle, 0);
    lv_obj_set_style_transform_pivot_x(img_obj, img_size / 2, 0);
    lv_obj_set_style_transform_pivot_y(img_obj, img_size / 2, 0);
  }
}

/**
 * @brief Re-positions and re-orients all UI elements based on the global `orientation` flag.
 */
void apply_orientation() {
  // --- Main Arcs ---
  if (orientation == 0) {  // Normal orientation (Top half)
    lv_arc_set_bg_angles(arc1, 180, 0);
    lv_arc_set_bg_angles(arc2, 180, 0);
    lv_arc_set_bg_angles(arc3, 180, 0);
    lv_arc_set_bg_angles(arc4, 0, 180);
    lv_arc_set_bg_angles(arc5, 0, 180);
  } else {  // Flipped orientation (Bottom half)
    lv_arc_set_bg_angles(arc1, 0, 180);
    lv_arc_set_bg_angles(arc2, 0, 180);
    lv_arc_set_bg_angles(arc3, 0, 180);
    lv_arc_set_bg_angles(arc4, 180, 0);
    lv_arc_set_bg_angles(arc5, 180, 0);
  }

  // --- Segmented Arcs (bottom_arcs) ---
  float r = 466 / 2.0f;
  float gap_px = 25.0f;
  float angle_gap = (gap_px / r) * (180.0f / PI);
  int num_arcs = 5;
  float total_gaps = (num_arcs - 1) * angle_gap;
  float total_arcs_angle = 180.0f - total_gaps;
  float each_arc_angle = total_arcs_angle / num_arcs;
  // Start at 0 for bottom half (normal), 180 for top half (flipped)
  float current_start = (orientation == 0) ? 0.0f : 180.0f;

  for (int i = 0; i < num_arcs; i++) {
    float current_end = current_start + each_arc_angle;
    lv_arc_set_bg_angles(bottom_arcs[i], (uint16_t)roundf(current_start), (uint16_t)roundf(current_end));
    current_start = current_end + angle_gap;
  }

  // --- Distance Label ---
  // Align to bottom half for normal, top half for flipped
  int32_t y_offset = (orientation == 0) ? 45 : -45;
  lv_obj_align(distance_label, LV_ALIGN_CENTER, 0, y_offset);

  // Set rotation based on orientation
  int16_t rotation_angle = (orientation == 1) ? 1800 : 0;  // 1800 = 180 degrees
  lv_obj_set_style_transform_angle(distance_label, rotation_angle, 0);
  lv_obj_set_style_transform_pivot_x(distance_label, lv_obj_get_width(distance_label) / 2, 0);
  lv_obj_set_style_transform_pivot_y(distance_label, lv_obj_get_height(distance_label) / 2, 0);
}

/**
 * @brief Sets the screen orientation by manually re-positioning all UI elements.
 * @param ori The orientation (0 for normal 0 degrees, 1 for flipped 180 degrees).
 */
void setOrientation(int ori) {
  if ((ori != 0 && ori != 1) || orientation == ori) return;
  orientation = ori;

  // Instead of using gfx->setRotation, we manually update all UI elements
  apply_orientation();

  lv_obj_invalidate(lv_scr_act());  // Force redraw to apply changes immediately
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
  palette_dark_grey = lv_color_make(20, 20, 20);   // New color for bottom arcs
  palette_heat_on = lv_color_make(255, 106, 0);    // Only for use later when logic is added for bottom arcs

  // Get the active screen object
  lv_obj_t *scr = lv_scr_act();

  // Set a black background for the screen and allow children to be drawn outside its bounds
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_flag(scr, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_add_event_cb(scr, screen_event_cb, LV_EVENT_ALL, NULL);

  // --- 1. Create Furthest Background Elements ---
  // These are the large, filled semicircles for each half. They must be created first.

  // --- Arc 4 (Heat Display Background) ---
  arc4 = lv_arc_create(scr);
  lv_obj_set_size(arc4, 336, 336);
  lv_obj_align(arc4, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_range(arc4, -90, 90);
  lv_arc_set_value(arc4, 90);
  lv_obj_remove_style(arc4, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc4, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc4, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc4, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc4, palette_dark_grey, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc4, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc4, 168, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc4, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc4, 0, LV_PART_MAIN);

  // --- Arc 3 (Main Display Background) ---
  arc3 = lv_arc_create(scr);
  lv_obj_set_size(arc3, 336, 336);
  lv_obj_align(arc3, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_range(arc3, -90, 90);
  lv_arc_set_value(arc3, 90);
  lv_obj_remove_style(arc3, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc3, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc3, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc3, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc3, palette_dark_green, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc3, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc3, 168, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc3, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc3, 0, LV_PART_MAIN);

  // --- 2. Create Mid-Ground Rings & Segments ---
  // These are drawn on top of the filled backgrounds.

  // --- Arc 5 (Heat Display Middle Ring) ---
  arc5 = lv_arc_create(scr);
  lv_obj_set_size(arc5, 370, 370);
  lv_obj_align(arc5, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_range(arc5, -90, 90);
  lv_arc_set_value(arc5, 90);
  lv_obj_remove_style(arc5, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc5, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc5, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc5, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc5, palette_light_grey, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc5, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc5, 72, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc5, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc5, 0, LV_PART_MAIN);

  // --- Arc 2 (Main Display Middle Ring) ---
  arc2 = lv_arc_create(scr);
  lv_obj_set_size(arc2, 370, 370);
  lv_obj_align(arc2, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_range(arc2, 0, 180);
  lv_arc_set_value(arc2, 180);
  lv_obj_remove_style(arc2, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc2, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc2, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc2, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc2, palette_green, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc2, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc2, 72, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc2, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc2, 0, LV_PART_MAIN);

  // --- Arc 1 (Main Display Outer Ring) ---
  arc1 = lv_arc_create(scr);
  lv_obj_set_size(arc1, 466, 466);
  lv_obj_align(arc1, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_range(arc1, -90, 90);
  lv_arc_set_value(arc1, 90);
  lv_obj_remove_style(arc1, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(arc1, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc1, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc1, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc1, palette_dark_green, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc1, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc1, 25, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc1, false, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc1, 0, LV_PART_MAIN);

  // --- Additional 5 arcs on the opposite side (Heat Display Outer Segments) ---
  for (int i = 0; i < 5; i++) {
    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 466, 466);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc, 0, 100);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc, palette_light_grey, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 25, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_MAIN);  // Ensure flat ends for background
    lv_obj_set_style_arc_color(arc, palette_heat_on, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 25, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);  // Ensure flat ends for indicator
    lv_arc_set_value(arc, 0);                                     // Start with value 0 (light grey)
    bottom_arcs[i] = arc;
  }

  // --- 3. Create Top-Most Foreground Elements ---
  // These must appear on top of everything else.

  // --- Arrow (Drawn directly with rotated polygon) ---
  arrow_img = lv_obj_create(scr);
  lv_obj_set_size(arrow_img, 466, 466);
  lv_obj_align(arrow_img, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(arrow_img, lv_color_make(0, 0, 0), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(arrow_img, 1, LV_PART_MAIN);
  lv_obj_set_style_border_opa(arrow_img, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_add_flag(arrow_img, LV_OBJ_FLAG_EVENT_BUBBLE | LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_add_event_cb(arrow_img, arrow_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
  lv_obj_set_user_data(arrow_img, (void *)(intptr_t)0);

  // --- Alert images positioned along an arc ---
  for (int i = 0; i < 5; i++) {
    images[i] = lv_img_create(scr);
    lv_obj_add_flag(images[i], LV_OBJ_FLAG_HIDDEN);  // Start hidden
    lv_obj_add_flag(images[i], LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  }

  // --- Distance Label ---
  distance_label = lv_label_create(scr);
  lv_label_set_text(distance_label, "200m");
  lv_obj_set_style_text_color(distance_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(distance_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_add_flag(distance_label, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

  // --- 4. Apply Initial Positions ---
  apply_orientation();
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

  // Initialize LVGL
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  // Allocate LVGL display buffers in DMA-capable internal memory
  uint32_t buf_rows = 120;
  buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  if (buf1 == NULL || buf2 == NULL) {
    USBSerial.println("Large internal buffer allocation failed! Falling back to minimal.");
    buf_rows = 40;
    if (buf1 == NULL) buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf2 == NULL) buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  }

  if (buf1 == NULL || buf2 == NULL) {
    USBSerial.println("Buffer allocation failed completely! Cannot proceed.");
    while (1)
      ;  // Halt
  }

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * buf_rows);

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

  // You can now call setOrientation(1) to flip the UI, and setOrientation(0) to flip it back.
  setOrientation(0);

  // Set initial heat level
  setHeat(4);

  // Set distance text
  updateDistance(666);

  // Set initial arrow angle
  updateAngle(0);

  updateAlerts(0, 0, 0, 0, 0);

  // Create a timer to handle the smooth animation of the arrow.
  // The animation logic is now inside this timer.
  lv_timer_create(arrow_update_timer, 10, NULL);  // Update every 10ms

  USBSerial.println("Setup done. Arc GUI is running.");
}

void loop() {
  static uint32_t last_angle_update = 0;
  //static int demo_angle = 0;
  static int alertCount = 0;
  if (millis() - last_angle_update > 2000) {  // Update every second
    last_angle_update = millis();
    //demo_angle = (demo_angle + 40) % 360;  // Move by 40 degrees
    //USBSerial.printf("Setting target angle to: %d\n", demo_angle);

    alertCount++;
    setHeat(alertCount);
    if (alertCount == 0) {
      updateAlerts(-1, -1, -1, -1, -1);
      updateAngle(179);
      updateDistance(-1);
    } else if (alertCount == 1) {
      updateAlerts(0, -1, -1, -1, -1);
      updateAngle(-90);
      updateDistance(1111);
    } else if (alertCount == 2) {
      updateAlerts(0, 0, -1, -1, -1);
      updateAngle(-30);
      updateDistance(930);
    } else if (alertCount == 3) {
      updateAlerts(0, 0, 0, -1, -1);
      updateAngle(0);
      updateDistance(400);
    } else if (alertCount == 4) {
      updateAlerts(0, 0, 0, 0, -1);
      updateAngle(45);
      updateDistance(300);
    } else if (alertCount == 5) {
      updateAlerts(0, 0, 0, 0, 0);
      updateAngle(75);
      updateDistance(150);
      alertCount = -1;
    }
  }

  lv_timer_handler();  // Let the GUI do its work
  delay(1);            // Minimal yield; adjust up if CPU overheats
}