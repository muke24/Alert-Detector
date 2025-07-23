/**
 * @file      AlertFinder.ino
 * @author    Peter Thompson
 * @brief     A simple GUI for the Waveshare 1.75" Round AMOLED Touch display,
 * showing three arcs and a rotating arrow.
 * (LVGL 8.4.0 API).
 * @version   2.11
 * @date      2025-07-23
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
  const float smoothing_factor = 0.1;  // Adjust for desired smoothness (0.1 = smoother, 0.5 = faster)

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
}

// Base stem points (quad: left head, right head, base right, base left; counter-clockwise)
// (Bottom half of arrow, scaled to 90% of original size)
static lv_point_t base_stem[4] = {
  { -38, 14 },  // {-42 * 0.9, 15 * 0.9}
  { 36, 14 },   // {40 * 0.9, 15 * 0.9}
  { 9, 111 },   // {10 * 0.9, 123 * 0.9}
  { -11, 111 }  // {-12 * 0.9, 123 * 0.9}
};

// Base head points (triangle: left head, tip, right head; counter-clockwise)
// (Top half of arrow, scaled to 90% of original size)
static lv_point_t base_head[3] = {
  { -74, 23 },  // {-82 * 0.9, 25 * 0.9}
  { 0, -113 },  // {0 * 0.9, -125 * 0.9}
  { 72, 23 }    // {80 * 0.9, 25 * 0.9}
};

// Outline stem points (offset 3 pixels outward from centroid {-1, 62.5})
static lv_point_t outline_stem[4] = {
  { -40, 12 },  // Offset from {-38, 14}
  { 38, 12 },   // Offset from {36, 14}
  { 10, 114 },  // Offset from {9, 111}
  { -12, 114 }  // Offset from {-11, 111}
};

// Outline head points (offset 3 pixels outward from centroid {-0.67, -22.33})
static lv_point_t outline_head[3] = {
  { -77, 25 },  // Offset from {-74, 23}
  { 0, -116 },  // Offset from {0, -113}
  { 75, 25 }    // Offset from {72, 23}
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
 * @brief Creates the GUI with three concentric semi-circle arcs and a rotating arrow.
 * Updated to use LVGL v8 styling API and style from reference script.
 */
void create_arc_gui() {
  // Define custom colors from the reference script
  lv_color_t palette_dark_green = lv_color_make(100, 193, 0);
  lv_color_t palette_green = lv_color_make(128, 255, 0);

  // Get the active screen object
  lv_obj_t *scr = lv_scr_act();

  // Set a black background for the screen
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  // Add a global event callback to the screen to capture all clicks
  lv_obj_add_event_cb(scr, screen_event_cb, LV_EVENT_ALL, NULL);

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

  // --- Arc 2 (Middle) ---
  lv_obj_t *arc2 = lv_arc_create(scr);
  lv_obj_set_size(arc2, 386, 386);
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
  lv_obj_set_style_arc_width(arc2, 50, LV_PART_INDICATOR);  // Increased thickness
  lv_obj_set_style_arc_rounded(arc2, false, LV_PART_INDICATOR);

  // --- Arc 1 (Outer) ---
  lv_obj_t *arc1 = lv_arc_create(scr);
  lv_obj_set_size(arc1, 441, 441);
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
  lv_obj_set_style_arc_width(arc1, 15, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc1, false, LV_PART_INDICATOR);  // Sharp ends

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
}

void setup() {
  USBSerial.begin(115200);

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

  // Allocate LVGL display buffers in DMA-capable memory
  buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * 40);

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

  // Create a timer to update the arrow rotation
  lv_timer_create(arrow_update_timer, 10, NULL);  // Update every 10ms for smooth rotation

  USBSerial.println("Setup done. Arc GUI is running.");
}

void loop() {
  lv_timer_handler();  // Let the GUI do its work
  delay(5);            // Yield to other tasks
}