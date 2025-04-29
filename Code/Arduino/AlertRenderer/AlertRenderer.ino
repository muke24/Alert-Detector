// Written by Peter Thompson

// TODO: Fix alert and icon naming confusion + Remove Heat level as its not an alert, but rather a calculation of multiple of the police alerts which is processed
// on the other device and does not need to be displayed.

// Keep in mind that an icon is a visual representation of an alert and they have the same indexes. 
// They basically mean the same thing, but icon is in context of this ESP32 device (using this script), and alert is in context of the other ESP32 device (using a different script).

// DATA OUT
// ALERT SELECTION:
// The user may select multiple icons (up to "maxIconsSelected") by holding the button for "secondsToSelectIcon" seconds which will select/deselect
// the center icon (when selected, the "selected" image will appear behind it). When a user selects/deselects an icon, we will then send data out from our rx/tx pins
// to our other ESP32 device which will send out all the icon indexes which were selected. The other ESP32 device will receive the icons which were selected and send
// a confirmation back via the rx/tx pins with the same int array we sent to it back to this device which we can confirm whether it matches our selected icon indexes
// (if it doesn't then we can retry until it does).

// DATA IN
// ALERT RECEIVING: 
// The other ESP32 device will be frequently looking for alerts within our area on Waze. Basically, the other ESP32 device sends the Waze API our location and Waze 
// returns it a list of alerts with their location. The other ESP32 will then process the respective data for each selected alert and send this ESP32 a dictionary
// consisting of the int index of the selected alert/icon and a float array holding its respective data.

// HANDING EACH ICONS RECEIVED DATA
// The float array received for each icon may be handled differently. Because of this, each icon needs to be processed differently using a different method.
// Below shows some different pseudocode examples of the received data from the other ESP32 device (with what the value represents in brackets and the dictionary value in square brackets):

// The dictionary can contain all of the different icons, with all but the last entry being used to rotate a corresponding arrow on the display 
// (if the icon supports it, the heat icon will not draw another arrow but is only used for the other ESP32 device).
// This example shows that police was detected and it is facing -85 degrees from our direction (which should now show a green arrow pointing towards the left). It also shows that a speed camera was detected
// Dictionary<[(index: police) 5, (data for this index) float[(direction: value between -90 to 90) -85f, (distance: in meters) 900f]], [(index: speedCamera) 6, (data for this index) float[(direction: value between -90 to 90) 70f], [(index: heat) 4, (data for this index) float[(heat level: value between 1 to 5) 3f]]>

// The selected icon indexes should be saved in storage so that they are kept when the device powers off.

// Here is the correlating data that each icon index expects and its value's usage:
// 0 (blockedLane): float - direction (rotates the corresponding arrow image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)
// 1 (closure): float - direction (rotates the corresponding arrow image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)
// 2 (crash): float - direction (rotates the arrow corresponding image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)
// 3 (hazard): float - direction (rotates the arrow corresponding image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)
// 4 (heat): null (nothing functionally happens on this device other than sending the other device data that this icon is selected) REMOVE THIS AS IT CAN BE FULLY PROCESSED IN THE OTHER SCRIPT
// 5 (police): float - direction (rotates the arrow corresponding image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)
// 6 (speedCamera): float - direction (rotates the arrow corresponding image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)
// 7 (traffic): float - direction (rotates the arrow corresponding image), float - distance (ONLY USED IF FIRST ELEMENT IN DICTIONARY: Displays as text on the display)

// USER INTERFACE STATES
// Startup State: 
// All UI should start off fully transparent and should fade in from fully transparent to fully visible as a startup animation unless stated otherwise. The arrow 
// should not be displayed unless the startup animation finishes and an alert is found and sent to this device. 
// The background image should fade in from fully transparent to fully visible as a startup animation. The icons in the icon carousel should bounce up from the 
// During Startup:
// Whilst a GPS and 4G connection are establishing, 
// On Startup Finished:

// Idle State:
// When an alert is found, the arrow should fade in rather than just appear.
// During 

/* Include our libraries */
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Encoder.h>
#include <Bounce2.h>

/* Include our images */
// Compass UI Images
#include "images/arrow.h"
#include "images/background.h"  // Rename to compass background for AI clarity

// Icon Images
#include "images/blockedLane.h" // 0 - icon index
#include "images/closure.h"     // 1 - icon index
#include "images/crash.h"       // 2 - icon index
#include "images/hazard.h"      // 3 - icon index
#include "images/heat.h"        // 4 - icon index: MAY NEED CHANGING
#include "images/police.h"      // 5 - icon index
#include "images/speedCamera.h" // 6 - icon index
#include "images/traffic.h"     // 7 - icon index

// Other UI
#include "images/selected.h"    // The icon background image which sits behind the selected icons.
#include "images/gps"           // The GPS status image. Displays red (and flashes in and out) when no gps connection is found, displays green when connection exists.
#include "images/network"       // The Network status image. Displays red (and flashes in and out) when no LTE or Wifi connection is found, displays green when connection exists.

// Max amount of icons we can select at a time.
int maxIconsSelected = 2;
int[] selectedIconIndexes = new float[0, 1, 2];
float secondsToSelectIcon = 3;

#define GFX_BL 8

Arduino_DataBus *bus = new Arduino_ESP32SPI(4 /* DC */, 10 /* CS */, 1 /* SCK */, 0 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, true /* IPS */);

#define ROTARY_ENCODER_A_PIN 7
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 9

Encoder myEnc(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN);
Bounce2::Button button = Bounce2::Button();

/* Functionality */
bool[] hasAlert = new bool[true, false, false]; // This will determine whether an arrow should be displayed. If array element is false, no corresponing arrow should display.

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

/* Button read callback (kept for later use) */
static void my_button_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  // Empty for now, to be implemented later
}

/* Display background, police icon, and arrow images */
void lv_example_page(void) {
  lv_obj_t *scr = lv_scr_act();

  /* Set the background color of the screen to black */
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  /* Background image */
  lv_obj_t *img_bg = lv_img_create(scr);
  lv_img_set_src(img_bg, &background_img);
  lv_obj_set_size(img_bg, screenWidth, screenHeight);
  lv_obj_align(img_bg, LV_ALIGN_CENTER, 0, 0);

  /* Police icon image */
  lv_obj_t *img_police = lv_img_create(scr);
  lv_img_set_src(img_police, &policeIcon_img);
  lv_obj_align(img_police, LV_ALIGN_CENTER, 0, 0);

  /* Arrow image on top */
  lv_obj_t *img_arrow = lv_img_create(scr);
  lv_img_set_src(img_arrow, &arrow_img);
  lv_obj_align(img_arrow, LV_ALIGN_CENTER, 0, 0);
}

void setup() {
  Serial.begin(115200);
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
  }

  /* Create LVGL task */
  xTaskCreatePinnedToCore(
    lvglTask, "lvglTask", 8192, NULL, 2, NULL, 1
  );

  Serial.println("Setup done");
}

void loop() {
  // Empty, as tasks handle everything
}

/* LVGL task */
void lvglTask(void *pvParameters) {
  Serial.println("lvglTask started");
  while (true) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}