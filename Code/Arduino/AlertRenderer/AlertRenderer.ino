#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Encoder.h>
#include <Bounce2.h>
#include <HardwareSerial.h>
#include "compassBackground.h"
#include "compassArrow.h"

#define GFX_BL 8
#define UART_RX_PIN 20 // or try 20
#define UART_TX_PIN 21 // or try 21

Arduino_DataBus *bus = new Arduino_ESP32SPI(4 /* DC */, 10 /* CS */, 1 /* SCK */, 0 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, true /* IPS */);

#define ROTARY_ENCODER_A_PIN 7
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 9

Encoder myEnc(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN);
Bounce2::Button button = Bounce2::Button();

// UART for communication with LILYGO
HardwareSerial SerialUART(1); // UART1
float currentAngle = 0; // Global angle from LILYGO

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

/* Arrow object */
static lv_obj_t *img_arrow;

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

/* Update arrow angle */
void update_arrow(lv_timer_t *timer) {
  if (currentAngle >= 0) {
    lv_img_set_angle(img_arrow, (int)(currentAngle * 10)); // LVGL uses 0.1 degree units
  }
}

/* Display background and arrow images */
void lv_example_page(void) {
  lv_obj_t *scr = lv_scr_act();

  /* Background image */
  lv_obj_t *img_bg = lv_img_create(scr);
  lv_img_set_src(img_bg, &compassBackground);
  lv_obj_set_size(img_bg, screenWidth, screenHeight / 2);
  lv_obj_align(img_bg, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_img_recolor(img_bg, lv_color_make(128, 255, 0), 0);
  lv_obj_set_style_img_recolor_opa(img_bg, LV_OPA_COVER, 0);

  /* Arrow image on top */
  img_arrow = lv_img_create(scr);
  lv_img_set_src(img_arrow, &compassArrow);
  lv_obj_align(img_arrow, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_img_recolor(img_arrow, lv_color_make(128, 255, 0), 0);
  lv_obj_set_style_img_recolor_opa(img_arrow, LV_OPA_COVER, 0);
}

void setup() {
  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
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
    lv_disp_drv_register(&disp_drv);
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
  // Empty, as tasks handle everything
}

/* UART task to read angle from LILYGO */
void uartTask(void *pvParameters) {
  while (true) {
    if (SerialUART.available()) {
      String data = SerialUART.readStringUntil('\n');
      Serial.println(data); // Debug anything which was received.
      if (data.startsWith("angle:")) {
        String angleStr = data.substring(6);
        currentAngle = angleStr.toFloat();
        
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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