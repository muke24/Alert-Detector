/**
 * @file      AlertRenderer.cpp
 * @author    Peter Thompson
 * @brief     Implementation of the UI rendering and management logic for the
 * Alert Finder. This file contains all LVGL object creation,
 * styling, and animations.
 * @version   3.01 (Corrected)
 * @date      2025-07-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "AlertRenderer.h"
#include <math.h>
#include "police.c" // Include the image data file
#include "HWCDC.h"  // For USB Serial
#include <esp_heap_caps.h>


#define PI 3.1415926535f // Define PI

// --- LVGL Buffers ---
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

// --- Touch Controller Variables ---
int16_t touch_x[5], touch_y[5];

// --- UI Object Handles ---
lv_obj_t *arrow_img;
lv_obj_t *distance_label;
lv_obj_t *arc1, *arc2, *arc3, *arc4, *arc5;
lv_obj_t *bottom_arcs[5];
lv_obj_t *images[5]; // Array to hold image objects

// --- State Variables ---
static float target_angle = 0;
static float current_angle = 0;
int orientation = 0; // 0: normal, 1: flipped
int current_heat_level = 0;

// --- UI Colors ---
lv_color_t palette_dark_grey;
lv_color_t palette_light_grey;
lv_color_t palette_heat_on;

// --- Dependencies for updateAlerts ---
LV_IMG_DECLARE(police);
const lv_img_dsc_t *alert_image_sources[] = {
  &police
};
const int NUM_ALERT_TYPES = sizeof(alert_image_sources) / sizeof(alert_image_sources[0]);

// Forward declaration for internal function
void apply_orientation();

/**
 * @brief Initializes LVGL, display buffers, and drivers.
 */
void renderer_init_lvgl() {
    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print);
#endif

    // Allocate LVGL display buffers
    uint32_t buf_rows = 120;
    buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * buf_rows * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    if (buf1 == NULL || buf2 == NULL) {
        USBSerial.println("Buffer allocation failed! Halting.");
        while (1);
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
}

// --- Animation Callbacks ---
void anim_set_opa_cb(void *var, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t *)var, v, 0);
}

static void screen_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        USBSerial.println("Screen clicked! (Button press action)");
    }
}

static void arrow_update_timer(lv_timer_t *timer) {
    const float smoothing_factor = 0.1f;
    float delta = target_angle - current_angle;

    if (delta > 1800) delta -= 3600;
    else if (delta < -1800) delta += 3600;

    if (fabs(delta) < 1.0f) {
        current_angle = target_angle;
    } else {
        current_angle += delta * smoothing_factor;
    }

    if (current_angle >= 3600) current_angle -= 3600;
    if (current_angle < 0) current_angle += 3600;

    lv_obj_set_user_data(arrow_img, (void *)(intptr_t)(int16_t)current_angle);
    lv_obj_invalidate(arrow_img);
}

// --- Arrow Polygon Data ---
static lv_point_t base_stem[4] = { { -38, 11 }, { 36, 11 }, { 9, 111 }, { -11, 111 } };
static lv_point_t outline_stem[4] = { { -40, 9 }, { 38, 9 }, { 10, 114 }, { -12, 114 } };
static lv_point_t base_head[3] = { { -74, 21 }, { 0, -119 }, { 72, 21 } };
static lv_point_t outline_head[3] = { { -77, 23 }, { 0, -122 }, { 75, 23 } };

static void arrow_draw_cb(lv_event_t *e) {
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
    lv_obj_t *obj = lv_event_get_target(e);
    int16_t angle = (int16_t)(intptr_t)lv_obj_get_user_data(obj);

    if (angle < 900 || angle > 2700) {
        if (orientation == 1) {
            angle = (angle + 1800) % 3600;
        }

        int16_t cos_a = lv_trigo_cos(angle / 10);
        int16_t sin_a = lv_trigo_sin(angle / 10);
        lv_point_t center = { (int16_t)(screenWidth / 2), (int16_t)(screenHeight / 2) };
        int32_t pivot_y = 111;

        lv_draw_rect_dsc_t outline_dsc;
        lv_draw_rect_dsc_init(&outline_dsc);
        outline_dsc.bg_color = lv_color_black();
        outline_dsc.bg_opa = LV_OPA_COVER;

        lv_point_t rotated_outline_stem[4];
        for (int i = 0; i < 4; i++) {
            int32_t dx = outline_stem[i].x;
            int32_t dy = outline_stem[i].y;
            int32_t dx_ = dx;
            int32_t dy_ = dy - pivot_y;
            rotated_outline_stem[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
            rotated_outline_stem[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
        }
        lv_draw_polygon(draw_ctx, &outline_dsc, rotated_outline_stem, 4);

        lv_point_t rotated_outline_head[3];
        for (int i = 0; i < 3; i++) {
            int32_t dx = outline_head[i].x;
            int32_t dy = outline_head[i].y;
            int32_t dx_ = dx;
            int32_t dy_ = dy - pivot_y;
            rotated_outline_head[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
            rotated_outline_head[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
        }
        lv_draw_polygon(draw_ctx, &outline_dsc, rotated_outline_head, 3);

        lv_draw_rect_dsc_t poly_dsc;
        lv_draw_rect_dsc_init(&poly_dsc);
        poly_dsc.bg_color = lv_color_make(128, 255, 0);
        poly_dsc.bg_opa = LV_OPA_COVER;

        lv_point_t rotated_stem[4];
        for (int i = 0; i < 4; i++) {
            int32_t dx = base_stem[i].x;
            int32_t dy = base_stem[i].y;
            int32_t dx_ = dx;
            int32_t dy_ = dy - pivot_y;
            rotated_stem[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
            rotated_stem[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
        }
        lv_draw_polygon(draw_ctx, &poly_dsc, rotated_stem, 4);

        lv_point_t rotated_head[3];
        for (int i = 0; i < 3; i++) {
            int32_t dx = base_head[i].x;
            int32_t dy = base_head[i].y;
            int32_t dx_ = dx;
            int32_t dy_ = dy - pivot_y;
            rotated_head[i].x = center.x + (int16_t)((dx_ * cos_a - dy_ * sin_a) / LV_TRIGO_SIN_MAX);
            rotated_head[i].y = center.y + (int16_t)((dx_ * sin_a + dy_ * cos_a) / LV_TRIGO_SIN_MAX);
        }
        lv_draw_polygon(draw_ctx, &poly_dsc, rotated_head, 3);
    }
}

// --- Public API Functions ---

void setHeat(int heatLevel) {
    if (heatLevel < 0) heatLevel = 0;
    if (heatLevel > 5) heatLevel = 5;

    int prev_heat_level = current_heat_level;
    current_heat_level = heatLevel;
    int delta = abs(heatLevel - prev_heat_level);
    if (delta == 0) return;

    int anim_time = 500 / delta;

    for (int i = 0; i < 5; i++) {
        int target_value = (i < heatLevel) ? 100 : 0;
        int current_value = lv_arc_get_value(bottom_arcs[i]);
        if (current_value != target_value) {
            lv_anim_del(bottom_arcs[i], (lv_anim_exec_xcb_t)lv_arc_set_value);
            uint32_t delay = 0;
            if (heatLevel > prev_heat_level) {
                delay = (i - prev_heat_level) * anim_time;
            } else {
                delay = (prev_heat_level - 1 - i) * anim_time;
            }

            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, bottom_arcs[i]);
            lv_anim_set_values(&a, current_value, target_value);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_arc_set_value);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
            lv_anim_set_time(&a, anim_time);
            lv_anim_set_delay(&a, delay);
            lv_anim_start(&a);
        }
    }
}

void updateDistance(int meters) {
    if (meters != -1) {
        int rounded_meters = round(meters / 50.0) * 50;
        char distance_text[16];
        if (rounded_meters < 1000) {
            snprintf(distance_text, sizeof(distance_text), "%dm", rounded_meters);
        } else {
            float kilometers = rounded_meters / 1000.0f;
            snprintf(distance_text, sizeof(distance_text), "%.2fkm", kilometers);
        }
        lv_label_set_text(distance_label, distance_text);
    } else {
        lv_label_set_text(distance_label, "");
    }
}

void updateAngle(int angle) {
    angle = angle % 360;
    if (angle < 0) angle += 360;
    target_angle = angle * 10;
}

void updateAlerts(int alertIndex0, int alertIndex1, int alertIndex2, int alertIndex3, int alertIndex4) {
    int input_indices[5] = { alertIndex0, alertIndex1, alertIndex2, alertIndex3, alertIndex4 };
    int active_indices[5];
    int num_active_alerts = 0;

    for (int i = 0; i < 5; i++) {
        if (input_indices[i] != -1) {
            active_indices[num_active_alerts++] = input_indices[i];
        }
    }

    for (int i = 0; i < 5; i++) {
        if (images[i]) {
            lv_anim_del(images[i], NULL);
            lv_obj_add_flag(images[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (num_active_alerts == 0) return;

    const float image_arc_r = 150.0f;
    const int center_x = screenWidth / 2;
    const int center_y = screenHeight / 2;
    const int img_size = 64;
    const uint32_t anim_duration = 700;
    const uint32_t anim_stagger = 100;
    const float spans[] = { 0.0f, 0.0f, 40.0f, 70.0f, 100.0f, 120.0f };
    float total_span_deg = spans[num_active_alerts];
    float center_angle_deg = (orientation == 0) ? 90.0f : 270.0f;
    float start_angle_deg = center_angle_deg - total_span_deg / 2.0f;
    float angle_step_deg = (num_active_alerts > 1) ? total_span_deg / (num_active_alerts - 1) : 0;

    int32_t anim_start_x = center_x + (int32_t)(image_arc_r * cosf(0.0f)) - (img_size / 2);
    int32_t anim_start_y = center_y + (int32_t)(image_arc_r * sinf(0.0f)) - (img_size / 2);

    for (int i = 0; i < num_active_alerts; i++) {
        int alert_type_index = active_indices[i];
        lv_obj_t *img_obj = images[i];

        if (!img_obj || alert_type_index < 0 || alert_type_index >= NUM_ALERT_TYPES) continue;

        lv_obj_set_size(img_obj, img_size, img_size);
        lv_img_set_src(img_obj, alert_image_sources[alert_type_index]);
        lv_obj_set_pos(img_obj, anim_start_x, anim_start_y);
        lv_obj_set_style_opa(img_obj, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(img_obj, LV_OBJ_FLAG_HIDDEN);

        float target_angle_rad = (start_angle_deg + i * angle_step_deg) * (PI / 180.0f);
        int32_t target_x = center_x + (int32_t)(image_arc_r * cosf(target_angle_rad)) - (img_size / 2);
        int32_t target_y = center_y + (int32_t)(image_arc_r * sinf(target_angle_rad)) - (img_size / 2);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, img_obj);
        lv_anim_set_time(&a, anim_duration);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_set_delay(&a, i * anim_stagger);

        lv_anim_set_values(&a, anim_start_x, target_x);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
        lv_anim_start(&a);

        lv_anim_set_values(&a, anim_start_y, target_y);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_start(&a);

        lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
        lv_anim_set_exec_cb(&a, anim_set_opa_cb);
        lv_anim_set_time(&a, anim_duration / 2);
        lv_anim_start(&a);

        int16_t rotation_angle = (orientation == 1) ? 1800 : 0;
        lv_obj_set_style_transform_angle(img_obj, rotation_angle, 0);
        lv_obj_set_style_transform_pivot_x(img_obj, img_size / 2, 0);
        lv_obj_set_style_transform_pivot_y(img_obj, img_size / 2, 0);
    }
}

void apply_orientation() {
    if (orientation == 0) {
        lv_arc_set_bg_angles(arc1, 180, 0);
        lv_arc_set_bg_angles(arc2, 180, 0);
        lv_arc_set_bg_angles(arc3, 180, 0);
        lv_arc_set_bg_angles(arc4, 0, 180);
        lv_arc_set_bg_angles(arc5, 0, 180);
    } else {
        lv_arc_set_bg_angles(arc1, 0, 180);
        lv_arc_set_bg_angles(arc2, 0, 180);
        lv_arc_set_bg_angles(arc3, 0, 180);
        lv_arc_set_bg_angles(arc4, 180, 0);
        lv_arc_set_bg_angles(arc5, 180, 0);
    }

    float r = screenWidth / 2.0f;
    float gap_px = 25.0f;
    float angle_gap = (gap_px / r) * (180.0f / PI);
    int num_arcs = 5;
    float total_arcs_angle = 180.0f - ((num_arcs - 1) * angle_gap);
    float each_arc_angle = total_arcs_angle / num_arcs;
    float current_start = (orientation == 0) ? 0.0f : 180.0f;

    for (int i = 0; i < num_arcs; i++) {
        float current_end = current_start + each_arc_angle;
        lv_arc_set_bg_angles(bottom_arcs[i], (uint16_t)roundf(current_start), (uint16_t)roundf(current_end));
        current_start = current_end + angle_gap;
    }

    int32_t y_offset = (orientation == 0) ? 45 : -45;
    lv_obj_align(distance_label, LV_ALIGN_CENTER, 0, y_offset);

    int16_t rotation_angle = (orientation == 1) ? 1800 : 0;
    lv_obj_set_style_transform_angle(distance_label, rotation_angle, 0);
    lv_obj_set_style_transform_pivot_x(distance_label, lv_obj_get_width(distance_label) / 2, 0);
    lv_obj_set_style_transform_pivot_y(distance_label, lv_obj_get_height(distance_label) / 2, 0);
}

void setOrientation(int ori) {
    if ((ori != 0 && ori != 1) || orientation == ori) return;
    orientation = ori;
    apply_orientation();
    lv_obj_invalidate(lv_scr_act());
}

void create_arc_gui() {
    lv_color_t palette_dark_green = lv_color_make(55, 107, 0);
    lv_color_t palette_green = lv_color_make(128, 255, 0);
    palette_light_grey = lv_color_make(30, 30, 30);
    palette_dark_grey = lv_color_make(20, 20, 20);
    palette_heat_on = lv_color_make(255, 106, 0);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_add_event_cb(scr, screen_event_cb, LV_EVENT_ALL, NULL);

    // --- Background Arcs ---
    arc4 = lv_arc_create(scr);
    lv_obj_set_size(arc4, 336, 336);
    lv_obj_align(arc4, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc4, -90, 90);
    lv_arc_set_value(arc4, 90);
    lv_obj_remove_style(arc4, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc4, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc4, palette_dark_grey, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc4, 168, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc4, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc4, 0, LV_PART_MAIN);

    arc3 = lv_arc_create(scr);
    lv_obj_set_size(arc3, 336, 336);
    lv_obj_align(arc3, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc3, -90, 90);
    lv_arc_set_value(arc3, 90);
    lv_obj_remove_style(arc3, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc3, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc3, palette_dark_green, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc3, 168, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc3, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc3, 0, LV_PART_MAIN);

    // --- Middle Rings ---
    arc5 = lv_arc_create(scr);
    lv_obj_set_size(arc5, 370, 370);
    lv_obj_align(arc5, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc5, -90, 90);
    lv_arc_set_value(arc5, 90);
    lv_obj_remove_style(arc5, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc5, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc5, palette_light_grey, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc5, 72, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc5, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc5, 0, LV_PART_MAIN);

    arc2 = lv_arc_create(scr);
    lv_obj_set_size(arc2, 370, 370);
    lv_obj_align(arc2, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc2, 0, 180);
    lv_arc_set_value(arc2, 180);
    lv_obj_remove_style(arc2, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc2, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc2, palette_green, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc2, 72, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc2, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc2, 0, LV_PART_MAIN);

    // --- Outer Rings ---
    arc1 = lv_arc_create(scr);
    lv_obj_set_size(arc1, 466, 466);
    lv_obj_align(arc1, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc1, -90, 90);
    lv_arc_set_value(arc1, 90);
    lv_obj_remove_style(arc1, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc1, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc1, palette_dark_green, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc1, 25, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc1, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc1, 0, LV_PART_MAIN);

    for (int i = 0; i < 5; i++) {
        bottom_arcs[i] = lv_arc_create(scr);
        lv_obj_set_size(bottom_arcs[i], 466, 466);
        lv_obj_align(bottom_arcs[i], LV_ALIGN_CENTER, 0, 0);
        lv_arc_set_range(bottom_arcs[i], 0, 100);
        lv_arc_set_value(bottom_arcs[i], 0);
        lv_obj_remove_style(bottom_arcs[i], NULL, LV_PART_KNOB);
        lv_obj_clear_flag(bottom_arcs[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_arc_color(bottom_arcs[i], palette_light_grey, LV_PART_MAIN);
        lv_obj_set_style_arc_width(bottom_arcs[i], 25, LV_PART_MAIN);
        lv_obj_set_style_arc_rounded(bottom_arcs[i], false, LV_PART_MAIN);
        lv_obj_set_style_arc_color(bottom_arcs[i], palette_heat_on, LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(bottom_arcs[i], 25, LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(bottom_arcs[i], false, LV_PART_INDICATOR);
    }

    // --- Foreground Elements ---
    arrow_img = lv_obj_create(scr);
    lv_obj_set_size(arrow_img, screenWidth, screenHeight);
    lv_obj_align(arrow_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(arrow_img, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(arrow_img, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_add_flag(arrow_img, LV_OBJ_FLAG_EVENT_BUBBLE | LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_add_event_cb(arrow_img, arrow_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_set_user_data(arrow_img, (void *)(intptr_t)0);

    for (int i = 0; i < 5; i++) {
        images[i] = lv_img_create(scr);
        lv_obj_add_flag(images[i], LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    }

    distance_label = lv_label_create(scr);
    lv_label_set_text(distance_label, "");
    lv_obj_set_style_text_color(distance_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(distance_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_add_flag(distance_label, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    // --- Initial Setup ---
    apply_orientation();

    // Create the timer for the arrow animation
    lv_timer_create(arrow_update_timer, 10, NULL);
}
