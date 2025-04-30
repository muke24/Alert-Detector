// Written by Peter Thompson

// TODO: Fix alert and icon naming confusion.
// CHANGE ALL "ICON" NAMES TO "ALERT".

// Keep in mind that an icon is a visual representation of an alert and they use the same indexes. 
// They basically mean the same thing, but icon is in context of this ESP32 device (using this script), and alert is in context of the other ESP32 device (using a different script).

// DATA IN
// ALERT RECEIVING: 
// The other ESP32 device will be frequently looking for alerts within our area on Waze. Basically, the other ESP32 device sends the Waze API our location and Waze 
// returns it a list of alerts with their location. The other ESP32 will then process the data for each alert and send this ESP32 a dictionary
// consisting of the int index of the alerts and a float array holding each alert's data.

// HANDLING EACH ALERTS RECEIVED DATA
// The float array received for each icon may be handled differently. Because of this, each icon should be processed differently using a different method for code clarity.
// Shared logic can be bundled up in another method which all of these methods can use, such as the compass updating logic (because all alerts will use direction and distance).
// All alerts will use direction and distance, however some may use another value. For example, this can be seen with a speedCamera alert and a traffic alert. 
// The speedCamera will only receive the values of direction and distance, whilst the traffic alert will receive the values of direction, distance and also level. 
// Each of these will be parameters in each alerts method. These methods will be used to control elements such the compass (the angle of the arrow, the color of 
// the compass background and arrow depending on what type of alert was detected keeping in mind that the compass background will be the color of the closest alert).
// For now, I am ignoring the level parameter because I plan to add that manually later, so nothing will happen for now if that parameter is received.

// Below shows a pseudocode example of the received data from the other ESP32 device (with what the value represents in brackets and the dictionary value in square brackets):
// The dictionary can contain all of the different alerts if they were detected on the other device, however we will only display up to 2 different arrows at once 
// (which are the first 2 "selectedIcons", and the alert which is closer will determine which arrow will be layered on top of the other).
// This example shows that a police alert was detected and it is facing -85 degrees from our direction, is 700 meters away and has a level of 3.
// It also shows that a speedCamera alert was detected and it is facing 70 degrees from our direction, is 400 meters away and has a level of 2.
// It also shows that a traffic alert was detected and it is facing 20 degrees from our direction, is 900 meters away and has a level of 4.
// {4: [-85, 700, 3], 5: [70, 400, 2], 6: [20, 900, 4]}

// The selected icon indexes should be saved in storage so that they are kept when the device powers off.

// Here is the correlating data that each icon index expects and its parameters usage:
// 0 (blockedLane): float - direction, float - distance, float level (converted to int).
// 1 (closure): float - direction, float - distance, float level (converted to int).
// 2 (crash): float - direction, float - distance, float level (converted to int).
// 3 (hazard): float - direction, float - distance, float level (converted to int).
// 4 (police): float - direction, float - distance, float level (converted to int).
// 5 (speedCamera): float - direction, float - distance, float level (converted to int).
// 6 (traffic): float - direction, float - distance, float level (converted to int).

// Here is what the parameters do (received from the float values within the float array of the dictionary):
// direction: Rotates the corresponding arrow image. Used with all alerts.
// distance: Determines which layer the corresponding arrow image will be on if it is displayed. Used with all alerts.
// level: The heat level for an alert, calculated on the other ESP32 by determining the amount of alerts within the general area. Updates the heat text and heat bars. 

// COLORS
// Each alert will have a different color which will be used for the compass arrow and background. This allows the user to differentiate between alerts using the compass 
// arrow and compass background color. Below is the RGB colors for each alert:
// 0 (blockedLane): (R: 224, G: 70, B: 65) - red.
// 1 (closure): (R: 255, G: 35, B: 196) - pink.
// 2 (crash): (R: 58, G: 228, B: 255) - blue.
// 3 (hazard): (R: 255, G: 255, B: 0) - yellow.
// 4 (police): (R: 128, G: 255, B: 0) - green.
// 5 (speedCamera): (R: 255, G: 174, B: 0) - orange.
// 6 (traffic): (R: 149, G: 66, B: 221) - purple.

// USER INTERFACE STATES
// STARTUP STATE: (Animate to idle state)
// Ignore for now, we can add animations later. I want to make sure the GUI is setup correctly before I animate it.

// IDLE STATE:

// UI
// Compass Images: The compass is located on the top half of the device. It includes two images, the compassBackground image and the compassArrow image. The top center 
// of the arrow should be located 2 pixels below the top center of the screen, and it should pivot from the center of the screen when it gets rotated. The arrow should
// not be visible if no alerts exist. The top left of compassBackground image should be at x 0, y 0. The color of the compassBackground and compassArrow images should
// change with the currently detected alerts. 

// Keep in mind, "selectedAlerts" holds the indexes of our selected alerts in order of their importance (which can be changed by the user).
// For example, it may hold something like [5, 4, 6], which means the speedCamera, police, and traffic alerts are selected and their importance is in that same order.
// This means that when we retreive the alert data from the other ESP32, we will only pick out the data which is relevant if it exists (if it doesn't exist then the 
// alert was never detected from the other ESP32 and we only pick out the data which does exist and is apart of our "selectedAlerts"). The data from the other ESP32 
// will be received as a dictionary as mentioned with the dictionary pseudocode earlier. So if the dictionary received from the other ESP32 was: 
// "{2: [-50, 300, 3], 4: [-85, 700, 3], 5: [70, 400, 2], 6: [20, 900, 4]}", then we should filter and save this data in a dictionary on this ESP32.
// This would look like "{5: [70, 400, 2], 4: [-85, 700, 3], 6: [20, 900, 4]}" because our selected alerts were [5, 4, 6]. By doing this, the user can prioritise
// the alerts which matters more to them if they coexist, and cut out alerts completely which they might not find useful. Alongside this, I would like a float value
// which can actually balance the priority against the distance of each alert. For example, the user might find that they want alerts with the priority of [5, 4, 6]

// Status Images: The status images are located on the sides of the arrow. They are both 32x16 images. The GPS status image should be on the left of the arrow, and the network
// status image should be on the right of the arrow. The bottom of each image should be one pixel above the center of the screen. The arrow and both status images should 
// not both appear at the same time, so visibility will not be an issue when the arrow is visible and is rotated. The top left of GPS status image should be located at 
// x 62, y 103. The top left of the network image should be located at x 146, y 103. 

/* Include our libraries */
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Encoder.h>
#include <Bounce2.h>

/* Include our images */
// Compass UI Images
#include "images/compassArrow.h"
#include "images/compassBackground.h"

// Alert Icon Images
#include "images/blockedLane.h" // 0 - icon index
#include "images/closure.h"     // 1 - icon index
#include "images/crash.h"       // 2 - icon index
#include "images/hazard.h"      // 3 - icon index
#include "images/police.h"      // 4 - icon index
#include "images/speedCamera.h" // 5 - icon index
#include "images/traffic.h"     // 6 - icon index

//#include "images/heat.h"        // 7 - icon index: For later development. Ignore for now.

// Other UI
#include "images/selected.h"    // The icon background image which sits behind the selected icons.
#include "images/gps"           // The GPS status image. Displays red (and flashes in and out) when no gps connection is found, disappears when connection exists.
#include "images/network"       // The Network status image. Displays red (and flashes in and out) when no LTE or Wifi connection is found, disappears when connection exists.


  // These are the alerts we want to track. The user may select any amount of 
float selectMode = 2.5;         //How long the button needs to be pressed for 

#define GFX_BL 8

Arduino_DataBus *bus = new Arduino_ESP32SPI(4 /* DC */, 10 /* CS */, 1 /* SCK */, 0 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, true /* IPS */);

#define ROTARY_ENCODER_A_PIN 7
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 9

Encoder myEnc(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN);
Bounce2::Button button = Bounce2::Button();

/* Functionality */
IndexKeyValue alertData[7];


/* Screen resolution */
static uint32_t screenWidth = 240;
static uint32_t screenHeight = 240;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf1;
lv_color_t *disp_draw_buf2;
lv_disp_drv_t disp_drv;
lv_group_t *group;

// SETUP //
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
///////////////////////////////////////////////////////////////

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
void DisplayGUI(void) {
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

void InitDisplay()
{
#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, LOW);
#endif
  gfx->begin(80000000);
  Serial.println("GFX initialized");

  lv_init();
  delay(10);
  Serial.println("LVGL initialized");
}

void Draw()
{
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

    DisplayGUI();
    Serial.println("Idle page loaded");
  }

  /* Create LVGL task */
  xTaskCreatePinnedToCore(
    lvglTask, "lvglTask", 8192, NULL, 2, NULL, 1
  );
}

void InitControls()
{
   /* Initialize button */
  button.attach(ROTARY_ENCODER_BUTTON_PIN, INPUT);
  button.interval(5);
  Serial.println("Button initialized");

  /* Initialize encoder */
  myEnc.write(0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  /* Initialize  */
  InitDisplay();
  
  InitControls();
 
  Draw();

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

struct AlertData
{
  float angle;    // Angle of the compass arrow.
  float distance; // Distance of the alert.
  int level;      // Level of the alert.

  AlertData(float _angle, float _distance, int _level) {
    angle = _angle;
    distance = _distance;
    level = _level;
  };
};

struct Alert
{
  int alertIndex; // Icon/alert index.
  AlertData data; // Associated alert data.

  Alert(int _alertIndex, float _angle, float _distance, int _level) {
    alertIndex = _alertIndex;
    data = new AlertData(_angle, _distance, _level);
  };
};

// Distance Profiles allow the user to select which alerts can be active within a certain distance.
// For example, I may only want to show certain alerts within 200m, whilst I may want to show different alerts within 500m (but above 200m).
struct DistanceProfile
{
  int distance;         // Less than or equal to this distance. In meters.
  int allowedAlerts[7]; // The alerts allowed within this distance.

  DistanceProfile(int _distance, int _allowedAlerts[7])
  {
    distance = _distance;
    allowedAlerts = _allowedAlerts;
  }
};

DistanceProfile distanceProfiles[7] = new DistanceProfile(200, {0, });

}