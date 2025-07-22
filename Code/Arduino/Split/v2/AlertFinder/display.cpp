#include "display.h"
#include "config.h"
#include "global_types.h"
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =========================================================================
// WAVESHARE DISPLAY PINOUT & OBJECTS
// =========================================================================
#define GFX_BL 12
#define GFX_DC 32
#define GFX_CS 19
#define GFX_SCK 18
#define GFX_MOSI 23
#define GFX_RST 33

Arduino_DataBus *bus = new Arduino_ESP32SPI(GFX_DC, GFX_CS, GFX_SCK, GFX_MOSI, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_RST, 0 /* rotation */, true /* IPS */);

// =========================================================================
// LVGL GLOBALS & WIDGETS
// =========================================================================
static uint32_t screenWidth = 240;
static uint32_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf1;
static lv_color_t *disp_draw_buf2;
static lv_disp_drv_t disp_drv;

// GUI Objects
static lv_obj_t *directionTxt;
static lv_obj_t *scr;
static lv_obj_t *arc1, *arc2, *arc3;
static lv_obj_t *source_canvas, *canvas;

// <<< FIX: Declare canvas buffers as pointers instead of static arrays
static lv_color_t *source_cbuf = NULL;
static lv_color_t *dest_cbuf = NULL;

// GUI Constants
const int dimention = 240;
const int arc1_width = 15, arc1_size = dimention - 25;
const int arc2_width = 35, arc2_size = dimention - 80;
const int arc3_width = 65, arc3_size = dimention - 130;
const int arc_start = 0, arc_end = 180;
const int compass_max_angle = 90;
const int arrow_width = 120, arrow_height = 128;
const int arrow_point_count = 7;

// Color Palette
lv_color_t palette_black = LV_COLOR_MAKE(0, 0, 0);
lv_color_t palette_dark_green = LV_COLOR_MAKE(100, 193, 0);
lv_color_t palette_green = LV_COLOR_MAKE(128, 255, 0);


// =========================================================================
// LVGL & DISPLAY FUNCTIONS
// =========================================================================

/* Display flushing callback */
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

/* Update compass arrow angle based on the global alertAngle */
void update_arrow(lv_timer_t *timer) {
    static float displayedAngle = 0.0;
    float targetAngle;

    taskENTER_CRITICAL(&timerMux);
    targetAngle = alertAngle;
    taskEXIT_CRITICAL(&timerMux);

    if (targetAngle >= -180.0 && targetAngle <= 180.0) {
        float delta = targetAngle - displayedAngle;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;
        displayedAngle += delta * 0.1;

        if (displayedAngle > 180.0) displayedAngle -= 360.0;
        if (displayedAngle < -180.0) displayedAngle += 360.0;

        lv_canvas_fill_bg(canvas, lv_color_make(0, 0, 0), LV_OPA_TRANSP);
        lv_img_dsc_t *src_img = lv_canvas_get_img(source_canvas);
        lv_canvas_transform(canvas, src_img, (int16_t)(displayedAngle * 10), LV_IMG_ZOOM_NONE, 0, 0, arrow_width / 2, arrow_height / 2, true);
        
        lv_obj_clear_flag(canvas, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_fmt(directionTxt, "%.1f", displayedAngle);
    } else {
        lv_obj_add_flag(canvas, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(directionTxt, "No Data");
    }
}

void draw_gui_elements() {
    scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, palette_black, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    // --- Compass Arcs ---
    arc1 = lv_arc_create(scr);
    lv_obj_set_size(arc1, arc1_size, arc1_size);
    lv_obj_align(arc1, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(arc1, arc_end, arc_start);
    lv_arc_set_range(arc1, -compass_max_angle, compass_max_angle);
    lv_arc_set_value(arc1, compass_max_angle);
    lv_obj_set_style_arc_color(arc1, palette_dark_green, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc1, arc1_width, LV_PART_INDICATOR);
    lv_obj_remove_style(arc1, NULL, LV_PART_KNOB | LV_PART_MAIN);

    arc2 = lv_arc_create(scr);
    lv_obj_set_size(arc2, arc2_size, arc2_size);
    lv_obj_align(arc2, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(arc2, arc_end, arc_start);
    lv_arc_set_range(arc2, -compass_max_angle, compass_max_angle);
    lv_arc_set_value(arc2, compass_max_angle);
    lv_obj_set_style_arc_color(arc2, palette_green, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc2, arc2_width, LV_PART_INDICATOR);
    lv_obj_remove_style(arc2, NULL, LV_PART_KNOB | LV_PART_MAIN);

    arc3 = lv_arc_create(scr);
    lv_obj_set_size(arc3, arc3_size, arc3_size);
    lv_obj_align(arc3, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(arc3, arc_end, arc_start);
    lv_arc_set_range(arc3, -compass_max_angle, compass_max_angle);
    lv_arc_set_value(arc3, compass_max_angle);
    lv_obj_set_style_arc_color(arc3, palette_dark_green, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc3, arc3_width, LV_PART_INDICATOR);
    lv_obj_remove_style(arc3, NULL, LV_PART_KNOB | LV_PART_MAIN);

    // --- Arrow Canvas Setup ---
    source_canvas = lv_canvas_create(scr);
    // <<< FIX: Allocate buffer memory from the heap at runtime
    source_cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(arrow_width, arrow_height), MALLOC_CAP_DMA);
    lv_canvas_set_buffer(source_canvas, source_cbuf, arrow_width, arrow_height, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_add_flag(source_canvas, LV_OBJ_FLAG_HIDDEN);
    
    lv_point_t arrow_points[] = { {36, 127}, {47, 127}, {59, 74}, {83, 74}, {42, 0}, {0, 74}, {24, 74} };
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = palette_green;
    line_dsc.width = 1;
    line_dsc.opa = LV_OPA_COVER;
    
    for (int y = 0; y < arrow_height; y++) {
        int min_x = arrow_width, max_x = 0;
        bool intersect = false;
        for (int i = 0; i < arrow_point_count; i++) {
            lv_point_t p1 = arrow_points[i];
            lv_point_t p2 = arrow_points[(i + 1) % arrow_point_count];
            if (((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) && (p2.y - p1.y != 0)) {
                int x = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                intersect = true;
            }
        }
        if (intersect) {
            lv_point_t fill_points[2] = {{min_x, y}, {max_x, y}};
            lv_canvas_draw_line(source_canvas, fill_points, 2, &line_dsc);
        }
    }

    canvas = lv_canvas_create(scr);
    // <<< FIX: Allocate buffer memory from the heap at runtime
    dest_cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(arrow_width, arrow_height), MALLOC_CAP_DMA);
    lv_canvas_set_buffer(canvas, dest_cbuf, arrow_width, arrow_height, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(canvas, LV_OBJ_FLAG_HIDDEN);

    // --- Angle Label ---
    directionTxt = lv_label_create(scr);
    lv_label_set_text(directionTxt, "No Data");
    lv_obj_align(directionTxt, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_text_color(directionTxt, lv_color_white(), 0);
}

/* LVGL task to handle rendering */
void lvglTask(void *pvParameters) {
    Serial.println("LVGL task started");
    while (true) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =========================================================================
// PUBLIC INTERFACE FUNCTIONS
// =========================================================================

void initDisplay() {
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    
    // --- TEST 1: Check if the GFX driver initializes successfully ---
    if (!gfx->begin()) {
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("!!! GFX FAILED TO INITIALIZE         !!!");
        Serial.println("!!! CHECK WIRING (especially RST, CS)!!! ");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        return; // Stop if we can't talk to the display
    }
    Serial.println("GFX initialized successfully.");

    // --- TEST 2: Draw directly to the screen to verify hardware and GFX library ---
    Serial.println("Performing direct GFX draw test...");
    gfx->fillScreen(BLACK);
    gfx->setCursor(10, 20);
    gfx->setTextColor(RED);
    gfx->setTextSize(2);
    gfx->println("GFX TEST: RED");
    gfx->setTextColor(GREEN);
    gfx->println("GFX TEST: GREEN");
    gfx->setTextColor(BLUE);
    gfx->println("GFX TEST: BLUE");
    gfx->drawRect(20, 100, 200, 100, WHITE);
    Serial.println("Direct GFX draw test complete. You should see text and a box.");
    delay(3000); // Pause for 3 seconds to see the result

    lv_init();
    Serial.println("LVGL initialized");

    disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 40, MALLOC_CAP_DMA);
    disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 40, MALLOC_CAP_DMA);

    if (!disp_draw_buf1 || !disp_draw_buf2) {
        Serial.println("LVGL disp_draw_buf allocate failed!");
        return;
    }

    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, screenWidth * 40);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    Serial.println("Display driver registered");

    draw_gui_elements();
    Serial.println("GUI elements drawn");

    lv_timer_create(update_arrow, 100, NULL);
}

void startDisplayTask() {
    xTaskCreatePinnedToCore(lvglTask, "lvglTask", 8192, NULL, 2, NULL, 1);
}