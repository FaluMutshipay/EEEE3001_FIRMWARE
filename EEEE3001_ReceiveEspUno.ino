/* 
 * Receiver (Robot ESP32 Layer feeding the Arduino Uno via Serial2)
 */

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  
#include <vector>

#define ESPNOW_WIFI_CHANNEL 6

#define BEND_MIN 0.0f
#define BEND_MAX 90.0f

// Struct layout matches the transmitter's memory structure perfectly
typedef struct {
  float bend[3];     // Index 0=Index, 1=Middle, 2=Ring
  float stretch[3];  
  float heading;     
} gloveData;

gloveData receivedData;
volatile bool newDataFlag = false;

// Custom Peer class definition matching your working template
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) 
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
  ~ESP_NOW_Peer_Class() {}

  bool add_peer() {
    if (!add()) return false;
    return true;
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
    if (len == sizeof(gloveData)) {
      memcpy(&receivedData, data, sizeof(gloveData));
      newDataFlag = true;
    }
  }
};

std::vector<ESP_NOW_Peer_Class *> masters;

// Your working dynamic address catcher function
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (!new_master->add_peer()) {
      delete new_master;
      return;
    }
    masters.push_back(new_master);
    if (len == sizeof(gloveData)) {
      memcpy(&receivedData, data, sizeof(gloveData));
      newDataFlag = true;
    }
  }
}

// Custom bound clamping math
int mapServoAngle(float value, float inMin, float inMax, int outMin, int outMax) {
  float mapped = (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
  if (mapped < outMin) mapped = outMin;
  if (mapped > outMax) mapped = outMax;
  return (int)mapped;
}

void setup() {
  Serial.begin(115200); // Laptop debug window

  // Pin 16 = RX2 (Unused), Pin 17 = TX2 -> Connects to Arduino Pin 0 (RX)
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }

  if (!ESP_NOW.begin()) {
    delay(5000);
    ESP.restart();
  }

  // Bind the dynamic peer tracking system that handles your transmitter
  ESP_NOW.onNewPeer(register_new_master, nullptr);
  Serial.println("Setup Complete. Intercepting Broadcast Envelopes...");
}

void loop() {
  if (newDataFlag) {
    newDataFlag = false;

    // --- MAP FLOATS TO SAFE SERVO DEGREES (0 - 180) ---
    // 1. Base (Channel 0) tracks IMU compass heading
    int baseAngle     = mapServoAngle(receivedData.heading, 0.0f, 360.0f, 0, 180);
    
    // 2. Main Arm joints map finger values to specific movements
    int shoulderAngle = mapServoAngle(receivedData.bend[0], BEND_MIN, BEND_MAX, 45, 135); // Index finger
    int elbowAngle    = mapServoAngle(receivedData.bend[1], BEND_MIN, BEND_MAX, 45, 135); // Middle finger
    
    // 3. Keep wrists centered for now as planned
    int wristPitch    = 90; 
    int wristRoll     = 90; 
    
    // 4. Claw (Channel 5) maps to your ring finger data profile
    int clawAngle     = mapServoAngle(receivedData.bend[2], BEND_MIN, BEND_MAX, 10, 170); // Ring finger

    // --- PACKET CONVERSION FOR SERIAL PIPELINE ---
    // Transmits via hardware pins: "A[base],[shoulder],[elbow],[pitch],[roll],[claw]\n"
    Serial2.print("A");
    Serial2.print(baseAngle);     Serial2.print(",");
    Serial2.print(shoulderAngle); Serial2.print(",");
    Serial2.print(elbowAngle);    Serial2.print(",");
    Serial2.print(wristPitch);    Serial2.print(",");
    Serial2.print(wristRoll);     Serial2.print(",");
    Serial2.print(clawAngle);
    Serial2.print("\n");

    // Local debug terminal monitoring output
    Serial.printf("SENT TO UNO -> B:%d | S:%d | E:%d | C:%d\n", baseAngle, shoulderAngle, elbowAngle, clawAngle);
  }
  delay(1);
}
