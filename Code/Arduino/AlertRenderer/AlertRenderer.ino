// AlertRenderer.ino
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Encoder.h>
#include <Bounce2.h>
#include <HardwareSerial.h>
//#include <stdio.h>
// Images
//#include "compassBackground.h"
//#include "compassBackground_0.h"
//#include "compassBackground_1.h"
//#include "compassArrow.h"
//#include "compassArrowOutline.h"
//#include "background.h"

// Pinout for this Viewe 1.28inch ESP32C3 push knob display module.
#define ADC_PIN 3 // GPIO03
#define RX_PIN 20  // GPIO20 RX
#define TX_PIN 21  // GPIO21 TX
#define UART_RECIEVE 0 // UART 0, 1 or 2 // USE UART 0 WITH A 10 PIN FPC EXTENSION BOARD. PIN 1: VCC, PIN 2: GND, PIN 6: RX, PIN 7: TX

#define GFX_BL 8

Arduino_DataBus *bus = new Arduino_ESP32SPI(4 /* DC */, 10 /* CS */, 1 /* SCK */, 0 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_NOT_DEFINED /* RST */, 3 /* rotation */, true /* IPS */);

#define ROTARY_ENCODER_A_PIN 7
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 9

Encoder myEnc(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN);
Bounce2::Button button = Bounce2::Button();

// UART for communication with LILYGO
HardwareSerial SerialUART(UART_RECIEVE);
float currentAngle = 999.0; // Global angle from LILYGO, initialized to sentinel value

/* Screen resolution */
static uint32_t screenWidth = 240;
static uint32_t screenHeight = 240;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf1;
lv_color_t *disp_draw_buf2;
lv_disp_drv_t disp_drv;
lv_group_t *group;

/* Last encoder value for tracking changes */
static int32_t last_encoder_value = 0;

/* Meter widget and its components */
static lv_obj_t *meter;
static lv_meter_scale_t *scale;
static lv_meter_indicator_t *indic;
// Direction Text
static lv_obj_t *directionTxt;

// Screen background
lv_obj_t *scr; 
// Screen size in pixels (1:1 aspect ratio because screen is round)
const int dimention = 240;

// Compass
lv_obj_t *arc1;
lv_obj_t *arc2;
lv_obj_t *arc3;
const int arc_start = 0; // Applied to all arcs
const int arc_end = 180;  // Applied to all arcs
const int arc1_width = 15;
const int arc1_size = dimention - 25;
const int arc2_width = 35;
const int arc2_size = dimention - 80;
const int arc3_width = 65;
const int arc3_size = dimention - 130;
const int compass_max_angle = 90;

// Compass Arrow
lv_obj_t *canvas; // Canvas for the arrow
const int arrow_width = 84; // Adjusted to match actual canvas width
const int arrow_height = 128; // Adjusted to match actual canvas height
const int arrow_point_count = 7;

// Color palette
lv_color_t palette_black = LV_COLOR_MAKE(0, 0, 0);
lv_color_t palette_dark_green = LV_COLOR_MAKE(100, 193, 0);
lv_color_t palette_green = LV_COLOR_MAKE(128, 255, 0);

/* Display flushing */
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

/* Encoder read callback */
static void my_encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  int32_t current_encoder_value = myEnc.read() / 2;
  data->enc_diff = current_encoder_value - last_encoder_value;
  last_encoder_value = current_encoder_value;
  button.update();
  data->state = button.isPressed() ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* Update arrow angle using Meter widget */
void update_arrow(lv_timer_t *timer) {
    if (currentAngle >= -180.0 && currentAngle <= 180.0) { // Valid angle range
        float adjustedAngle = currentAngle;
        while (adjustedAngle < -180.0) adjustedAngle += 360.0; // Normalize to [-180, 180]
        while (adjustedAngle > 180.0) adjustedAngle -= 360.0;
        //lv_meter_set_indicator_value(meter, indic, (int32_t)adjustedAngle); // Set angle for meter needle
        //lv_obj_clear_flag(meter, LV_OBJ_FLAG_HIDDEN); // Show meter
        char angleStr[16]; // Buffer for string conversion
        snprintf(angleStr, sizeof(angleStr), "%.1f", adjustedAngle); // Convert float to string
        lv_label_set_text(directionTxt, angleStr); // Update label with angle
    } else {
        //lv_meter_set_indicator_value(meter, indic, 0); // Reset needle to 0
        //lv_obj_add_flag(meter, LV_OBJ_FLAG_HIDDEN); // Hide meter
        lv_label_set_text(directionTxt, "No Data"); // Update label to indicate no data
    }
}

void ApplyBackgroundColor()
{
  // Set background color to black
  lv_obj_set_style_bg_color(scr, palette_black, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
}

void DrawCompassBackground()
{
  // Create arc
  arc1 = lv_arc_create(scr);
  lv_obj_set_size(arc1, arc1_size, arc1_size); // Set arc size (230x230)
  lv_obj_align(arc1, LV_ALIGN_CENTER, 0, 0);  // Center the arc
  lv_arc_set_bg_angles(arc1, arc_end, arc_start); // Set background angles (0 to 180)
  lv_arc_set_range(arc1, -compass_max_angle, compass_max_angle); // Set range (-90 to 90)
  lv_arc_set_value(arc1, compass_max_angle); // Set value to max (90)

  // Style the arc
  lv_obj_set_style_arc_color(arc1, palette_black, LV_PART_MAIN); // Background arc color
  lv_obj_set_style_arc_opa(arc1, LV_OPA_COVER, LV_PART_MAIN); // Ensure background is opaque
  lv_obj_set_style_arc_color(arc1, palette_dark_green, LV_PART_INDICATOR); // Indicator color
  lv_obj_set_style_arc_opa(arc1, LV_OPA_COVER, LV_PART_INDICATOR); // Ensure indicator is opaque
  lv_obj_set_style_arc_width(arc1, arc1_width, LV_PART_INDICATOR); // Set indicator width to 15
  lv_obj_set_style_arc_rounded(arc1, false, LV_PART_INDICATOR); // Sharp ends for indicator
  lv_obj_remove_style(arc1, NULL, LV_PART_KNOB); // Remove knob

  // Create arc
  arc2 = lv_arc_create(scr);
  lv_obj_set_size(arc2, arc2_size, arc2_size); // Set arc size (230x230)
  lv_obj_align(arc2, LV_ALIGN_CENTER, 0, 0);  // Center the arc
  lv_arc_set_bg_angles(arc2, arc_end, arc_start); // Set background angles (0 to 180)
  lv_arc_set_range(arc2, -compass_max_angle, compass_max_angle); // Set range (-90 to 90)
  lv_arc_set_value(arc2, compass_max_angle); // Set value to max (90)

  // Style the arc
  lv_obj_set_style_arc_color(arc2, palette_black, LV_PART_MAIN); // Background arc color
  lv_obj_set_style_arc_opa(arc2, LV_OPA_COVER, LV_PART_MAIN); // Ensure background is opaque
  lv_obj_set_style_arc_color(arc2, palette_green, LV_PART_INDICATOR); // Indicator color
  lv_obj_set_style_arc_opa(arc2, LV_OPA_COVER, LV_PART_INDICATOR); // Ensure indicator is opaque
  lv_obj_set_style_arc_width(arc2, arc2_width, LV_PART_INDICATOR); // Set indicator width to 15
  lv_obj_set_style_arc_rounded(arc2, false, LV_PART_INDICATOR); // Sharp ends for indicator
  lv_obj_remove_style(arc2, NULL, LV_PART_KNOB); // Remove knob

  // Create arc
  arc3 = lv_arc_create(scr);
  lv_obj_set_size(arc3, arc3_size, arc3_size); // Set arc size (230x230)
  lv_obj_align(arc3, LV_ALIGN_CENTER, 0, 0);  // Center the arc
  lv_arc_set_bg_angles(arc3, arc_end, arc_start); // Set background angles (0 to 180)
  lv_arc_set_range(arc3, -compass_max_angle, compass_max_angle); // Set range (-90 to 90)
  lv_arc_set_value(arc3, compass_max_angle); // Set value to max (90)

  // Style the arc
  lv_obj_set_style_arc_color(arc3, palette_black, LV_PART_MAIN); // Background arc color
  lv_obj_set_style_arc_opa(arc3, LV_OPA_COVER, LV_PART_MAIN); // Ensure background is opaque
  lv_obj_set_style_arc_color(arc3, palette_dark_green, LV_PART_INDICATOR); // Indicator color
  lv_obj_set_style_arc_opa(arc3, LV_OPA_COVER, LV_PART_INDICATOR); // Ensure indicator is opaque
  lv_obj_set_style_arc_width(arc3, arc3_width, LV_PART_INDICATOR); // Set indicator width to 15
  lv_obj_set_style_arc_rounded(arc3, false, LV_PART_INDICATOR); // Sharp ends for indicator
  lv_obj_remove_style(arc3, NULL, LV_PART_KNOB); // Remove knob
}

// Method to set up the arrow on a canvas
void DrawArrow() {
    // Create canvas for the arrow
    canvas = lv_canvas_create(scr);
    lv_obj_set_size(canvas, arrow_width, arrow_height); // Set canvas size to 84x128
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 0); // Center the canvas

    // Allocate buffer for the canvas (RGBA format for transparency)
    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(arrow_width, arrow_height)];
    lv_canvas_set_buffer(canvas, cbuf, arrow_width, arrow_height, LV_IMG_CF_TRUE_COLOR_ALPHA);

    // Define arrow points
    lv_point_t arrow_points[] = {
    {36, 127}, // Base right
    {47, 127}, // Base left
    {59, 74},  // Inner left
    {83, 74},  // Outer left
    {42, 0},   // Tip (top)
    {0, 74},   // Outer right
    {24, 74},  // Inner right
    {36, 127}  // Back to start
};

    // Define fill descriptor
    lv_draw_line_dsc_t line_dsc_fill;
    lv_draw_line_dsc_init(&line_dsc_fill);
    line_dsc_fill.color = palette_green; // Green fill color
    line_dsc_fill.width = 1;             // Thinner lines for fill
    line_dsc_fill.opa = LV_OPA_COVER;    // Fully opaque

    // Define outline descriptor
    lv_draw_line_dsc_t line_dsc_outline;
    lv_draw_line_dsc_init(&line_dsc_outline);
    line_dsc_outline.color = palette_black; // Black outline color
    line_dsc_outline.width = 2;             // Slightly thicker outline
    line_dsc_outline.opa = LV_OPA_COVER;    // Fully opaque

    // Draw the fill by approximating with horizontal lines
    for (int y = 0; y < arrow_height; y += 1) {
        int x_left = arrow_width, x_right = 0;
        for (int i = 0; i < arrow_point_count; i++) {
            lv_point_t p1 = arrow_points[i];
            lv_point_t p2 = arrow_points[(i + 1) % arrow_point_count]; // Close the loop
            if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) {
                int x = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
                if (x < x_left) x_left = x;
                if (x > x_right) x_right = x;
            }
        }
        if (x_right > x_left) {
            lv_point_t fill_points[] = {{x_left, y}, {x_right, y}};
            lv_canvas_draw_line(canvas, fill_points, 2, &line_dsc_fill);
        }
    }

    // Draw the outline by connecting points with lines
    for (int i = 0; i < arrow_point_count; i++) {
        lv_point_t line_points[] = {arrow_points[i], arrow_points[(i + 1) % arrow_point_count]};
        lv_canvas_draw_line(canvas, line_points, 2, &line_dsc_outline);
    }
}

/* Display background and meter widget */
void lv_example_page(void) {
  scr = lv_scr_act();
  ApplyBackgroundColor();

  DrawCompassBackground();

  DrawArrow();

  /* Direction text label */
  directionTxt = lv_label_create(scr);
  char angleStr[16]; // Buffer for string conversion
  if (currentAngle >= -180.0 && currentAngle <= 180.0) {
    snprintf(angleStr, sizeof(angleStr), "%.1f", currentAngle); // Convert float to string
    lv_label_set_text(directionTxt, angleStr);
  } else {
    lv_label_set_text(directionTxt, "No Data");
  }
  lv_obj_align(directionTxt, LV_ALIGN_CENTER, 0, -20);
}

void setup() {
  Serial.begin(115200);
  // Initialize UART1 with default pins (GPIO20 RX, GPIO21 TX) to match LILYGO's 115200 baud, 8N1
  SerialUART.begin(115200, SERIAL_8N1/*, RX_PIN, TX_PIN*/);
  Serial.println("Setup started");

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, LOW);
#endif
  gfx->begin(80000000);
  Serial.println("GFX initialized");

  lv_init();
  delay(10);
  Serial.println("LVGL initialized");

  /* Initialize button */
  button.attach(ROTARY_ENCODER_BUTTON_PIN, INPUT);
  button.interval(5);
  Serial.println("Button initialized");

  /* Initialize encoder */
  myEnc.write(0);

  /* Allocate display buffers */
  disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 8, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 8, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (!disp_draw_buf1 && !disp_draw_buf2) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
    return;
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, screenWidth * screenHeight / 8);

    /* Initialize display driver */
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);      // Modified line: capture the display pointer
    lv_disp_set_rotation(disp, LV_DISP_ROT_270);            // Added line: set rotation to 270 degrees
    Serial.println("Display initialized");

    /* Initialize input device (encoder only) */
    group = lv_group_create();
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = my_encoder_read;
    lv_indev_t *indev_encoder = lv_indev_drv_register(&indev_drv);
    lv_indev_set_group(indev_encoder, group);
    Serial.println("Input devices initialized");

    lv_example_page();
    Serial.println("Example page loaded");

    /* Create timer to update arrow */
    lv_timer_create(update_arrow, 100, NULL);
  }

  /* Create UART task */
  xTaskCreatePinnedToCore(uartTask, "uartTask", 4096, NULL, 1, NULL, 0);
  /* Create LVGL task */
  xTaskCreatePinnedToCore(lvglTask, "lvglTask", 8192, NULL, 2, NULL, 1);

  Serial.println("Setup done");
}

void loop() {

}

/* UART task to read angle from LILYGO */
void uartTask(void *pvParameters) {
  while (true) {
    if (SerialUART.available() >= sizeof(float)) { // Check if at least 4 bytes are available
      float relativeAngle;
      SerialUART.readBytes((uint8_t*)&relativeAngle, sizeof(float)); // Read 4 bytes into float
      Serial.print("Received Relative Angle: ");
      Serial.print(relativeAngle, 1); // Print angle with 1 decimal place for debugging
      Serial.println("° (0° is ahead)");
      
      // Validate angle (LILYGO sends 999.0 for no alert)
      if (relativeAngle >= -180.0 && relativeAngle <= 180.0) {
        currentAngle = relativeAngle; // Update global angle for valid values
      } else {
        currentAngle = 999.0; // Set to sentinel value for invalid/no alert
        Serial.println("No valid police alert received (sentinel value).");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms between checks
  }
}

/* LVGL task */
void lvglTask(void *pvParameters) {
  Serial.println("lvglTask started");
  while (true) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}