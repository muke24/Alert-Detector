// AlertRenderer: Written by Peter Thompson.

// Include our libraries
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Encoder.h>
#include <Bounce2.h>

// Include our images
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
#include "images/status/gps.h"      // The GPS status image. Displays red (and flashes in and out) when no gps connection is found, disappears when connection exists.
#include "images/status/network.h"  // The Network status image. Displays red (and flashes in and out) when no LTE or Wifi connection is found, disappears when connection exists.

// Image declarations
LV_IMG_DECLARE(compassBackground);
LV_IMG_DECLARE(compassArrow);
LV_IMG_DECLARE(blockedLane);
LV_IMG_DECLARE(closure);
LV_IMG_DECLARE(crash);
LV_IMG_DECLARE(hazard);
LV_IMG_DECLARE(police);
LV_IMG_DECLARE(camera);
LV_IMG_DECLARE(traffic);
LV_IMG_DECLARE(heat);
LV_IMG_DECLARE(heatBar1);
LV_IMG_DECLARE(heatBar2);
LV_IMG_DECLARE(heatBar3);
LV_IMG_DECLARE(heatBar4);
LV_IMG_DECLARE(heatBar5);
LV_IMG_DECLARE(gps);
LV_IMG_DECLARE(network);

// The data that an alert holds.
struct AlertData
{
  float angle;    // Angle of the compass arrow.
  float distance; // Distance of the alert.
  int level;      // Level of the alert.

  AlertData(float _angle = 0.0, float _distance = 0.0, int _level = 0) : angle(_angle), distance(_distance), level(_level) {}
};

// Alert index and data.
struct Alert
{
  int alertIndex; // Alert index.
  AlertData data; // Associated alert data.

  // Default constructor
  Alert() : alertIndex(-1), data(0.0, 0.0, 0) {}
  
  // Parameterized constructor
  Alert(int _alertIndex, float _angle, float _distance, int _level) : alertIndex(_alertIndex), data(_angle, _distance, _level) {}
};

// Distance Profiles allow the user to select which alerts can be active within a certain distance of us and their priority.
// Allows the user to select x alerts within y meters of us with different priorities. For example, this allows the user to prioritise traffic alerts over police alerts
// within the distance of 200m - 500m, whilst police alerts may be prioritised over traffic alerts within the distance of 0m to 200m. In the case that police and traffic
// both exist at both 0m - 200m and 200m - 500m, the closer Distance Profile will be used. By doing this, the user can prioritise
// the alerts which matters more to them if they coexist, and cut out alerts completely which they might not find useful.
struct DistanceProfile
{
  int distance;                     // Less than or equal to this distance. In meters.
  Alert selectedAlerts[7];          // The alerts allowed within this distance. In order of priority.

  DistanceProfile(int _distance = 0, Alert* _selectedAlerts = nullptr) : distance(_distance) {
    if (_selectedAlerts) {
      for (int i = 0; i < 7; i++) {
        selectedAlerts[i] = _selectedAlerts[i];
      }
    }
    // If _selectedAlerts is nullptr, selectedAlerts is already default-constructed
  }
};

// Functional Variables (variables used for the functionality of this project)
float selectMode = 2.5;           // How long the button needs to be pressed for to exit the idle state and enter the (replace me) state.
DistanceProfile userProfile[3] = {
  DistanceProfile(), DistanceProfile(), DistanceProfile()
};   // These are the user selected distance profiles which hold the selected alerts for each distance.
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

// Global UI objects (accessible for updates)
static lv_obj_t *compass_bg, *arrow1, *arrow2, *gps_status, *network_status;
static lv_obj_t *alertIconObjs[MaxAlerts];
static lv_obj_t *heatBars[5], *heatIcon, *heatLabel, *distanceLabel;

// Alert data to display (updated elsewhere)
Alert alertsToDisplay[MaxArrows];
int numAlertsToDisplay = 0;
bool hasGPSConnection = true;    // Example default, update based on actual status
bool hasNetworkConnection = true;
int heatLevel = 0;
int distance = 0;

// Alert colors
static lv_color_t alertColors[MaxAlerts] = {
    lv_color_make(224, 70, 65),   // blockedLane - red
    lv_color_make(255, 35, 196),  // closure - pink
    lv_color_make(58, 228, 255),  // crash - blue
    lv_color_make(255, 255, 0),   // hazard - yellow
    lv_color_make(128, 255, 0),   // police - green
    lv_color_make(255, 174, 0),   // camera - orange
    lv_color_make(149, 66, 221)   // traffic - purple
};

// Function to reset alertsToDisplay to default values
void resetAlerts() {
  for (int i = 0; i < MaxArrows; i++) {
    alertsToDisplay[i] = Alert(); // Default-constructed Alert
  }
  numAlertsToDisplay = 0;
}

// Display flushing
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

/* Update GUI based on current state */
static void update_gui_task(lv_timer_t *timer) {
  char buf[16]; // Buffer for string conversion
  if (numAlertsToDisplay > 0) {
    // Update compass background color to primary alert
    lv_obj_set_style_img_recolor(compass_bg, alertColors[alertsToDisplay[0].alertIndex], 0);
    lv_obj_set_style_img_recolor_opa(compass_bg, LV_OPA_COVER, 0);

    // Update arrow1
    lv_obj_clear_flag(arrow1, LV_OBJ_FLAG_HIDDEN);
    lv_img_set_angle(arrow1, (int)(alertsToDisplay[0].data.angle * 10));
    lv_obj_set_style_img_recolor(arrow1, alertColors[alertsToDisplay[0].alertIndex], 0);

    // Handle second arrow only if it exists
    if (arrow2 != nullptr && numAlertsToDisplay > 1) {
      int closerIndex = alertsToDisplay[0].data.distance < alertsToDisplay[1].data.distance ? 0 : 1;
      int fartherIndex = 1 - closerIndex;
      lv_obj_clear_flag(arrow2, LV_OBJ_FLAG_HIDDEN);
      lv_img_set_angle(arrow2, (int)(alertsToDisplay[fartherIndex].data.angle * 10));
      lv_obj_set_style_img_recolor(arrow2, alertColors[alertsToDisplay[fartherIndex].alertIndex], 0);
    } else {
      if (arrow2 != nullptr) {
        lv_obj_add_flag(arrow2, LV_OBJ_FLAG_HIDDEN);
      }
    }

    // Hide status images when alerts are present
    lv_obj_add_flag(gps_status, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(network_status, LV_OBJ_FLAG_HIDDEN);

    // Update heat level and distance
    heatLevel = alertsToDisplay[0].data.level;
    distance = alertsToDisplay[0].data.distance;
    if (numAlertsToDisplay > 1 && arrow2 != nullptr) {
      distance = min(alertsToDisplay[0].data.distance, alertsToDisplay[1].data.distance);
    }
    snprintf(buf, sizeof(buf), "%d", heatLevel);
    lv_label_set_text(heatLabel, buf);
    snprintf(buf, sizeof(buf), "%dm", distance);
    lv_label_set_text(distanceLabel, buf);
    for (int i = 0; i < 5; i++) {
      if (i < heatLevel) {
        lv_obj_clear_flag(heatBars[i], LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(heatBars[i], LV_OBJ_FLAG_HIDDEN);
      }
    }
  } else {
    // No alerts: gray compass, hide arrows, show status if no connection
    lv_obj_set_style_img_recolor(compass_bg, lv_color_hex(0x808080), 0);
    lv_obj_set_style_img_recolor_opa(compass_bg, LV_OPA_COVER, 0);
    lv_obj_add_flag(arrow1, LV_OBJ_FLAG_HIDDEN);
    if (arrow2 != nullptr) {
      lv_obj_add_flag(arrow2, LV_OBJ_FLAG_HIDDEN);
    }

    if (!hasGPSConnection) {
      lv_obj_clear_flag(gps_status, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(gps_status, LV_OBJ_FLAG_HIDDEN);
    }
    if (!hasNetworkConnection) {
      lv_obj_clear_flag(network_status, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(network_status, LV_OBJ_FLAG_HIDDEN);
    }

    lv_label_set_text(heatLabel, "0");
    lv_label_set_text(distanceLabel, "0m");
    for (int i = 0; i < 5; i++) {
      lv_obj_add_flag(heatBars[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

/* Display background, police icon, and arrow images */
void DisplayGUI(void) {
  lv_obj_t *scr = lv_scr_act();

  // Compass background
  compass_bg = lv_img_create(scr);
  lv_img_set_src(compass_bg, &compassBackground);
  lv_obj_set_pos(compass_bg, 0, 0);
  lv_obj_set_style_img_recolor(compass_bg, lv_color_hex(0x808080), 0); // Default gray
  lv_obj_set_style_img_recolor_opa(compass_bg, LV_OPA_COVER, 0);

  // Compass arrows
  arrow1 = lv_img_create(scr);
  lv_img_set_src(arrow1, &compassArrow);
  int arrow_w = compassArrow.header.w;
  int arrow_h = compassArrow.header.h;
  lv_img_set_pivot(arrow1, arrow_w / 2, arrow_h);
  lv_obj_set_pos(arrow1, 120 - arrow_w / 2, 120 - arrow_h);
  lv_obj_set_style_img_recolor(arrow1, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_img_recolor_opa(arrow1, LV_OPA_COVER, 0);
  lv_obj_add_flag(arrow1, LV_OBJ_FLAG_HIDDEN);

  // arrow2 = lv_img_create(scr);
  // lv_img_set_src(arrow2, &compassArrow);
  // lv_img_set_pivot(arrow2, arrow_w / 2, arrow_h);
  // lv_obj_set_pos(arrow2, 120 - arrow_w / 2, 120 - arrow_h);
  // lv_obj_set_style_img_recolor(arrow2, lv_color_hex(0xFFFFFF), 0);
  // lv_obj_set_style_img_recolor_opa(arrow2, LV_OPA_COVER, 0);
  // lv_obj_add_flag(arrow2, LV_OBJ_FLAG_HIDDEN);

  // Status images
  gps_status = lv_img_create(scr);
  lv_img_set_src(gps_status, &gps);      // The GPS status image. Displays red (and flashes in and out) when no gps connection is found, disappears when connection exists.
  lv_obj_set_pos(gps_status, 62, 103);
  lv_obj_add_flag(gps_status, LV_OBJ_FLAG_HIDDEN);

  network_status = lv_img_create(scr);
  lv_img_set_src(network_status, &network);  // The Network status image. Displays red (and flashes in and out) when no LTE or Wifi connection is found, disappears when connection exists.
  lv_obj_set_pos(network_status, 146, 103);
  lv_obj_add_flag(network_status, LV_OBJ_FLAG_HIDDEN);

  // Alert icons (icon index follows the same index scheme as alert index)
  const lv_img_dsc_t* alertIcons[MaxAlerts] = {
    &blockedLane, // 0 - icon index
    &closure,     // 1 - icon index
    &crash,       // 2 - icon index
    &hazard,      // 3 - icon index
    &police,      // 4 - icon index
    &camera,      // 5 - icon index
    &traffic      // 6 - icon index
  };
  int icon_width = 24; // Example; adjust based on actual size
  int total_width = MaxAlerts * icon_width + (MaxAlerts - 1) * 6; // 6px spacing
  int start_x = (screenWidth - total_width) / 2;
  for (int i = 0; i < MaxAlerts; i++) {
    alertIconObjs[i] = lv_img_create(scr);
    lv_img_set_src(alertIconObjs[i], alertIcons[i]);
    lv_obj_set_pos(alertIconObjs[i], start_x + i * (icon_width + 6), 150);
  }

  // Define heat bar positions
  lv_coord_t heatBarX[5] = {10, 25, 79, 162, 203};
  lv_coord_t heatBarY[5] = {120, 171, 210, 176, 120};

  // Array of heat bar images (assuming these are defined elsewhere)
  const lv_img_dsc_t* heatBarImages[5] = { &heatBar1, &heatBar2, &heatBar3, &heatBar4, &heatBar5 };

  // Create and position heat bars
  for (int i = 0; i < 5; i++) {
    heatBars[i] = lv_img_create(scr);
    lv_img_set_src(heatBars[i], heatBarImages[i]);
    lv_obj_set_pos(heatBars[i], heatBarX[i], heatBarY[i]);  // Set specific positions
    lv_obj_add_flag(heatBars[i], LV_OBJ_FLAG_HIDDEN);       // Initially hidden
  }

  // Heat icon and label
  heatIcon = lv_img_create(scr);
  lv_img_set_src(heatIcon, &heat); // Heat icon - (non indexed on purpose)
  lv_obj_set_pos(heatIcon, 100, 180);

  heatLabel = lv_label_create(scr);
  lv_label_set_text(heatLabel, "0");
  lv_obj_set_pos(heatLabel, 120, 180);

  // Distance label
  distanceLabel = lv_label_create(scr);
  lv_label_set_text(distanceLabel, "0m");
  lv_obj_set_pos(distanceLabel, 108, 220);
  lv_obj_align(distanceLabel, LV_ALIGN_CENTER, 0, 0); // Center horizontally

  // Start GUI update timer
  lv_timer_create(update_gui_task, 100, NULL);
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

  // Initialize alerts
  resetAlerts();

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
}