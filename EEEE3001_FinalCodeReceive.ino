/* 
 * Receiver (Robot Side)
 */

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  
#include <Adafruit_PWMServoDriver.h>
#include <vector>

#define ESPNOW_WIFI_CHANNEL 6

// Motorino Address
Adafruit_PWMServoDriver motorino = Adafruit_PWMServoDriver(0x40);

// Motorino Servo Channels
#define SERVO_INDEX_CH 0
#define SERVO_MIDDLE_CH 1
#define SERVO_RING_CH 2
#define SERVO_WRIST_CH 3

// PWM Pulse Lengths
#define SERVO_MIN 150 // 0 Degrees
#define SERVO_MAX 600 // 180 Degrees

// Angle Limits
#define INDEX_MIN 0
#define INDEX_MAX 170
#define MIDDLE_MIN 0
#define MIDDLE_MAX 170
#define RING_MIN 0
#define RING_MAX 170
#define WRIST_MIN 0
#define WRIST_MAX 180

#define BEND_MIN 0.0f
#define BEND_MAX 90.0f

typedef struct {
  float bend[3];     // Index 0=Thumb, 1=Index, 2=Middle
  float stretch[3];  
  float heading;     
} gloveData;

gloveData receivedData;
volatile bool newDataFlag = false;

// Convert angle (0-180) to PCA9685 Pulse Value
int angleToPulse(int angle) {
  return(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

int mapServoAngle(float value, float inMin, float inMax, float outMin, float outMax) {
  float mapped = (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
  if (mapped < outMin) {
    mapped = outMin;
  }
  if (mapped > outMax) {
    mapped = outMax;
  }
  return (int)mapped;
}    

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
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

int mapServoAngle(float value, float inMin, float inMax,  int outMin, int outMax) {
  float mapped = (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
  if (mapped < outMin) mapped = outMin;
  if (mapped > outMax) mapped = outMax;
  return (int)mapped;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  motorino.begin();
  motorino.setOscillatorFrequency(27000000); // Set Oscillator Frequency to 27MHz
  motorino.setPWMFreq(50); // Set PWM Frequency to 50Hz
  delay(10);

  // Centre Servos On Startup
  motorino.setPWM(SERVO_INDEX_CH, 0, angleToPulse(90));
  motorino.setPWM(SERVO_MIDDLE_CH, 0, angleToPulse(90));
  motorino.setPWM(SERVO_RING_CH, 0, angleToPulse(90));
  motorino.setPWM(SERVO_WRIST_CH, 0, angleToPulse(90));

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }

  if (!ESP_NOW.begin()) {
    delay(5000);
    ESP.restart();
  }

  ESP_NOW.onNewPeer(register_new_master, nullptr);
  Serial.println("Setup Complete. Intercepting Broadcast Envelopes...");
}

void loop() {
  if (newDataFlag) {
    newDataFlag = false;

    // Fixed: Pulling values out from individual array tracking slots [0], [1], [2]
    int indexAngle  = mapServoAngle(receivedData.bend[0], BEND_MIN, BEND_MAX, INDEX_MIN, INDEX_MAX);
    int middleAngle  = mapServoAngle(receivedData.bend[1], BEND_MIN, BEND_MAX, MIDDLE_MIN, MIDDLE_MAX);
    int ringAngle = mapServoAngle(receivedData.bend[2], BEND_MIN, BEND_MAX, RING_MIN, RING_MAX);
    int wristAngle  = mapServoAngle(receivedData.heading, 0.0f, 360.0f, WRIST_MIN, WRIST_MAX);

    motorino.setPWM(SERVO_INDEX_CH, 0, angleToPulse(indexAngle));
    motorino.setPWM(SERVO_MIDDLE_CH, 0, angleToPulse(middleAngle));
    motorino.setPWM(SERVO_RING_CH, 0, angleToPulse(ringAngle));
    motorino.setPWM(SERVO_WRIST_CH, 0, angleToPulse(wristAngle));

    Serial.print("INDEX:");  Serial.print(indexAngle);
    Serial.print(" MIDDLE:");  Serial.print(middleAngle);
    Serial.print(" RING:"); Serial.print(ringAngle);
    Serial.print(" WRIST:");  Serial.println(wristAngle);
  }
  delay(1);
}
