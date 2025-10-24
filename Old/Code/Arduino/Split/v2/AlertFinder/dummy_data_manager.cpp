#include "dummy_data_manager.h"
#include "global_types.h"
#include "alert_processor.h" // For processAlerts()
#include <ArduinoJson.h>

// Static JSON data that mimics a real API response from Waze.
// Locations are set around Sydney, NSW.
const char* dummyJsonData = R"({
  "alerts": [
    {
      "type": "POLICE",
      "subtype": "HIDDEN",
      "location": { "y": -33.8550, "x": 151.2110 },
      "street": "Bradfield Hwy"
    },
    {
      "type": "ROAD_CLOSED",
      "subtype": "ROAD_CLOSED_EVENT",
      "location": { "y": -33.8680, "x": 151.2050 },
      "street": "George St"
    },
    {
      "type": "POLICE",
      "subtype": "VISIBLE",
      "location": { "y": -33.8910, "x": 151.2770 },
      "street": "Campbell Parade"
    },
    {
      "type": "POLICE",
      "subtype": "HIDDEN",
      "location": { "y": -33.8750, "x": 151.1750 },
      "street": "Parramatta Rd"
    }
  ]
})";


void fetchDummyWazeData() {
    Serial.println("--- FETCHING DUMMY DATA ---");

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, dummyJsonData);
    
    if (error) {
        Serial.print("Dummy JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    JsonArray wazeAlerts = doc["alerts"];

    // Safely lock the mutex while modifying shared alert data
    taskENTER_CRITICAL(&timerMux);

    if (currentAlerts != nullptr) {
        delete[] currentAlerts;
        currentAlerts = nullptr;
    }

    alertCount = wazeAlerts.size();
    if (alertCount > 0) {
        currentAlerts = new Alert[alertCount];
        for (int i = 0; i < alertCount; i++) {
            currentAlerts[i].type = wazeAlerts[i]["type"].as<String>();
            currentAlerts[i].subtype = wazeAlerts[i]["subtype"].as<String>();
            currentAlerts[i].location.latitude = wazeAlerts[i]["location"]["y"].as<float>();
            currentAlerts[i].location.longitude = wazeAlerts[i]["location"]["x"].as<float>();
            currentAlerts[i].street = wazeAlerts[i]["street"].as<String>();
        }
    }
    
    taskEXIT_CRITICAL(&timerMux);
    
    // Process the newly "fetched" alerts
    processAlerts(); 
}