// AlertRenderer: Written by Peter Thompson.

// ALERT RECEIVING: 
// The other ESP32 device will be frequently looking for alerts within our area on Waze. Basically, the other ESP32 device sends the Waze API our location and Waze 
// returns it a list of alerts with their location. The other ESP32 will then process the data for each alert and send this ESP32 the data of all of the alerts which
// were found. We need to be able to check for any data from our RX port from the other ESP32 device. This data should be an Alert array (The Alert struct found near 
// the bottom of the script).

// We will only display up to "maxArrows" different arrows at once (which are the first 2 "selectedAlerts", and the alert which is closer will determine which arrow will be 
// layered on top of the other).

// The selected icon indexes should be saved in storage so that they are kept when the device powers off.

// Here is the correlating data that each alert icon index expects and its parameters usage:
// 0 (blockedLane): float - direction, float - distance, float level (converted to int).
// 1 (closure): float - direction, float - distance, float level (converted to int).
// 2 (crash): float - direction, float - distance, float level (converted to int).
// 3 (hazard): float - direction, float - distance, float level (converted to int).
// 4 (police): float - direction, float - distance, float level (converted to int).
// 5 (camera): float - direction, float - distance, float level (converted to int).
// 6 (traffic): float - direction, float - distance, float level (converted to int).

// Here is what the parameters do (from the float values within alertData):
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
// 5 (camera): (R: 255, G: 174, B: 0) - orange.
// 6 (traffic): (R: 149, G: 66, B: 221) - purple.

// USER INTERFACE STATES
// STARTUP STATE: (Animates to idle state). Ignore.
// Ignore the startup state for now.

// IDLE STATE (The state which shows our compass, status images (when applicable), the icons representing the alerts which have been found, and the heat level of our selected heatLevelAlert):

// UI
// Compass Images: The compass is located on the top half of the device. It includes two images, the compassBackground image and the compassArrow image. The top center 
// of the arrow should be located 2 pixels below the top center of the screen, and it should pivot from the center of the screen when it gets rotated. The arrow should
// not be visible if no alerts exist. The top left of compassBackground image should be at x 0, y 0. The color of the compassBackground and compassArrow images should
// change with the currently detected alerts. 
// Heat Bars (the 5 orange bars which span from the left, to the bottom and to the right of the screen): Amount of bars scale with the heat level. 
// The bar towards the left is the the first heat level (1) and the bar towards the right is the last (5). We should display each bar with what our selected
// heatLevelAlert's heat level is.

// Keep in mind, "selectedAlerts" within our DistanceProfiles holds the indexes of our selected alerts in order of their importance (which can be changed by the user) at each distance.
// For example, it may hold something like [5, 4, 6, -1, -1, -1, -1] (-1 indicates no alert is selected in that index), which means the camera, police, and traffic alerts are selected and their importance is in that same order.
// This means that when we retrieve the alert data from the other ESP32, we will only pick out the data which is relevant to our Distance Profiles if it exists. 

// Status Images: The status images are located on the sides of the arrow. They are both 32x16 images. The GPS status image should be on the left of the arrow, and the network
// status image should be on the right of the arrow. The bottom of each image should be one pixel above the center of the screen. The arrow and both status images should 
// not both appear at the same time, so visibility will not be an issue when the arrow is visible and is rotated. The top left of GPS status image should be located at 
// x 62, y 103. The top left of the network image should be located at x 146, y 103.

// OTHER STATES:
// There will be other states which will allow the user to customise their experience. These have not yet been included or designed.
// I am thinking of doing something like this: The user can hold the button down for a couple of seconds which will make this device go into a settings state.
// There will be multiple options, such as modifying the Distance Profiles (so the user can select the alerts which they want to see within each distance, from 
// 1 to 5 distance profiles), UI customisation (which will allow the user to change the colour theme, UI differentiations, amount of arrows to display which can only 
// be 1 or 2), device settings (changing the LED colour for the connected LED strip on the other ESP32 device and volume settings) and Alert Settings (Later in 
// development, each alert will have subtypes which are basically a more accurate representation of the alert, for example: a camera subtype may be Red Light Cameras,
// Speed Cameras, Stop Sign Cameras ect. Within Alert Settings, we can choose an alert and select or deselect the subtypes of it). For now, I will not include these for simplicity
// and clarity, and will work on it once the idle state is completed.

// COMPONENTS (In Context)
// ESP32:
// 1. ESP32C3 Dev Module
// 2. CPU Frequency: 160MHz (WiFi)
// 3. Flash Frequency: 80MHz
// 4. Flash Mode: QIO 80MHz
// 5. Flash Size: 4MB (32Mb)
// 6. Partition Scheme: Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)
// 7. PSRAM: NO

// Screen
// 1. Viewe 1.28 Inch Round screen
// 2. 240x240 Pixels
// 3. Button
// 4. Rotary encoder knob

/* Include our libraries */
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Encoder.h>
#include <Bounce2.h>
// Add more if needed.

/* Include our images */
// Compass UI Images
#include "images/compass/compassArrow.h"
#include "images/compass/compassBackground.h"

// Alert Icon Images (icon index follows the same index scheme as alert index)
#include "images/alerts/blockedLane.h" // 0 - icon index
#include "images/alerts/closure.h"     // 1 - icon index
#include "images/alerts/crash.h"       // 2 - icon index
#include "images/alerts/hazard.h"      // 3 - icon index
#include "images/alerts/police.h"      // 4 - icon index
#include "images/alerts/camera.h"      // 5 - icon index
#include "images/alerts/traffic.h"     // 6 - icon index

// Heat Images
#include "images/heat/heat.h"     // Heat icon - (non indexed on purpose)
#include "images/heat/heatBar1.h" // This visually displays heat level 1
#include "images/heat/heatBar2.h" // This visually displays heat level 2
#include "images/heat/heatBar3.h" // This visually displays heat level 3
#include "images/heat/heatBar4.h" // This visually displays heat level 4
#include "images/heat/heatBar5.h" // This visually displays heat level 5

// Status Images
#include "images/status/gps"      // The GPS status image. Displays red (and flashes in and out) when no gps connection is found, disappears when connection exists.
#include "images/status/network"  // The Network status image. Displays red (and flashes in and out) when no LTE or Wifi connection is found, disappears when connection exists.

// Functional Variables (variables used for the functionality of this project)
float selectMode = 2.5;           // How long the button needs to be pressed for to exit the idle state and enter the (replace me) state.
DistanceProfile userProfile[3];   // These are the user selected distance profiles which hold the selected alerts for each distance.
int heatLevelAlert = 0;           // The alert we will use to display its heat level (severity / intensity of the alert calculated from the other ESP32, e.g., 1 = low, 2 = low-medium, 3 = medium, 4 = high-medium, 5 = high).
int currentUiState = 1;           // The current page index to be displayed. Right now, only the Idle state exists.
// Add more if needed

#define MaxAlerts 7             // Total amount of alerts
#define MaxDistanceProfiles 5   // Maximum amount of distance profiles.
#define MaxArrows 2             // Maximum amount of compass arrows allowed to be displayed.
// Add more if needed

// Rendering
#define GFX_BL 8

Arduino_DataBus *bus = new Arduino_ESP32SPI(4 /* DC */, 10 /* CS */, 1 /* SCK */, 0 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, true /* IPS */);

#define ROTARY_ENCODER_A_PIN 7
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 9

Encoder myEnc(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN);
Bounce2::Button button = Bounce2::Button();

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

  // TODO: Create the idle state UI.
  
}

// Ignore for now as we are only getting the idle state to work. To be added later.
void ChangeUiState(int pageStateIndex)
{
  // TODO: Add logic to change the page.
  // 0 = Startup state
  // 1 = Idle state
  // More will be added later... Ignore 0 for now as we will focus on the Idle state.
}

// Initialise our display.
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

// The logic which allows our GUI to be displayed.
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

// Initialises our rotary encoder and button.
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
  // Debug
  Serial.begin(115200);
  Serial.println("Setup started");

  /* Initialize */
  InitDisplay();
  InitControls();
 
  // Draw GUI
  Draw();

  // Debug
  Serial.println("Setup done");
}

void loop() {
  // Empty, as tasks handle everything.
  // We might use this for animations later, however we won't animate anything yet.
}

/* LVGL task */
void lvglTask(void *pvParameters) {
  Serial.println("lvglTask started");
  while (true) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

// The data that an alert holds.
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

// Alert index and data.
struct Alert
{
  int alertIndex; // Alert index.
  AlertData data; // Associated alert data.

  Alert(int _alertIndex, float _angle, float _distance, int _level) {
    alertIndex = _alertIndex;
    data = AlertData(_angle, _distance, _level);
  };
};

// Distance Profiles allow the user to select which alerts can be active within a certain distance of us and their priority.
// Allows the user to select x alerts within y meters of us with different priorities. For example, this allows the user to prioritise traffic alerts over police alerts
// within the distance of 200m - 500m, whilst police alerts may be prioritised over traffic alerts within the distance of 0m to 200m. In the case that police and traffic
// both exist at both 0m - 200m and 200m - 500m, the closer Distance Profile will be used. By doing this, the user can prioritise
// the alerts which matters more to them if they coexist, and cut out alerts completely which they might not find useful.
struct DistanceProfile
{
  int distance;                     // Less than or equal to this distance. In meters.
  Alert selectedAlerts[MaxAlerts];  // The alerts allowed within this distance. In order of priority.

  DistanceProfile(int _distance, Alert _selectedAlerts[MaxAlerts])
  {
    distance = _distance;
    selectedAlerts = _selectedAlerts;
  };
};

}