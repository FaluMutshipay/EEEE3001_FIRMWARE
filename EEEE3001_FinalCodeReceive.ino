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

Adafruit_PWMServoDriver motorino = Adafruit_PWMServoDriver(0x40);

#define SERVO_INDEX_CH  0
#define SERVO_MIDDLE_CH 1
#define SERVO_RING_CH   2
#define SERVO_WRIST_CH  3

#define SERVO_MIN 150
#define SERVO_MAX 600

#define INDEX_MIN  0
#define INDEX_MAX  170
#define MIDDLE_MIN 0
#define MIDDLE_MAX 170
#define RING_MIN   0
#define RING_MAX   170
#define WRIST_MIN  0
#define WRIST_MAX  180

#define BEND_MIN 0.0f
#define BEND_MAX 90.0f

/* ESP-NOW Struct — must match transmitter exactly including seqNum */
typedef struct {
  float bend[3];
  float stretch[3];  
  float heading;   
  uint32_t seqNum;   // Sequence number for packet loss detection  
} gloveData;

gloveData receivedData;
volatile bool newDataFlag = false;

// Packet loss tracking variables (Single global definitions)
volatile uint32_t receivedCount  = 0;
volatile uint32_t droppedCount   = 0;
volatile uint32_t lastSeqNum     = 0;
volatile bool     firstPacket    = true;

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

int mapServoAngle(float value, float inMin, float inMax, int outMin, int outMax) {
  float mapped = (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
  if (mapped < outMin) mapped = outMin;
  if (mapped > outMax) mapped = outMax;
  return (int)mapped;
}

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
    // Only intercept and track statistics if within the active 10-second test window
    if (millis() < 10000) {
      if (len == sizeof(gloveData)) {
        memcpy(&receivedData, data, sizeof(gloveData));
        newDataFlag = true;

        receivedCount++;

        if (firstPacket) {
          // First packet received — initialise the sequence tracker
          lastSeqNum = receivedData.seqNum;
          firstPacket = false;
        } else {
          // Check for gaps in the sequence number
          uint32_t expected = lastSeqNum + 1;
          if (receivedData.seqNum != expected) {
            uint32_t missed = receivedData.seqNum - expected;
            droppedCount += missed;
            Serial.print("DROPPED ");
            Serial.print(missed);
            Serial.print(" packet(s) — expected SeqNum ");
            Serial.print(expected);
            Serial.print(" got ");
            Serial.println(receivedData.seqNum);
          }
          lastSeqNum = receivedData.seqNum;
        }

        // Print running packet loss statistics
        uint32_t totalExpected = receivedCount + droppedCount;
        float lossRate = (totalExpected > 0) ? (droppedCount * 100.0f / totalExpected) : 0.0f;

        Serial.print("SeqNum:");    Serial.print(receivedData.seqNum);  Serial.print(" ");
        Serial.print("Received:");  Serial.print(receivedCount);         Serial.print(" ");
        Serial.print("Dropped:");   Serial.print(droppedCount);          Serial.print(" ");
        Serial.print("Loss%:");     Serial.println(lossRate, 2);
      }
    }
  }
};

std::vector<ESP_NOW_Peer_Class *> masters;

void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(
      info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (!new_master->add_peer()) {
      delete new_master;
      return;
    }
    masters.push_back(new_master);
    if (millis() < 10000 && len == sizeof(gloveData)) {
      memcpy(&receivedData, data, sizeof(gloveData));
      newDataFlag = true;
    }
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  motorino.begin();
  motorino.setOscillatorFrequency(27000000);
  motorino.setPWMFreq(50);
  delay(10);

  motorino.setPWM(SERVO_INDEX_CH,  0, angleToPulse(90));
  motorino.setPWM(SERVO_MIDDLE_CH, 0, angleToPulse(90));
  motorino.setPWM(SERVO_RING_CH,   0, angleToPulse(90));
  motorino.setPWM(SERVO_WRIST_CH,  0, angleToPulse(90));

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
  // --- 10 Second Timer Packet Loss Summary ---
  static bool testComplete = false;

  if (millis() >= 10000) {
    if (!testComplete) {
      testComplete = true;
      uint32_t totalExpected = receivedCount + droppedCount;
      float lossRate = (totalExpected > 0) ? (droppedCount * 100.0f / totalExpected) : 0.0f;

      Serial.println("\n=================================");
      Serial.println("     10-SECOND TEST COMPLETE     ");
      Serial.print("  Packets Received: ");
      Serial.println(receivedCount);
      Serial.print("  Packets Dropped:  ");
      Serial.println(droppedCount);
      Serial.print("  Total Expected:   ");
      Serial.println(totalExpected);
      Serial.printf("  Packet Loss Rate: %.2f%%\n", lossRate);
      Serial.printf("  Throughput: %.2f packets/sec\n", receivedCount / 10.0);
      Serial.println("=================================");
    }
    return;
  }

  if (newDataFlag) {
    newDataFlag = false;

    int indexAngle  = mapServoAngle(receivedData.bend[0], BEND_MIN, BEND_MAX, INDEX_MIN,  INDEX_MAX);
    int middleAngle = mapServoAngle(receivedData.bend[1], BEND_MIN, BEND_MAX, MIDDLE_MIN, MIDDLE_MAX);
    int ringAngle   = mapServoAngle(receivedData.bend[2], BEND_MIN, BEND_MAX, RING_MIN,   RING_MAX);
    int wristAngle  = mapServoAngle(receivedData.heading, 0.0f, 360.0f, WRIST_MIN, WRIST_MAX);

    motorino.setPWM(SERVO_INDEX_CH,  0, angleToPulse(indexAngle));
    motorino.setPWM(SERVO_MIDDLE_CH, 0, angleToPulse(middleAngle));
    motorino.setPWM(SERVO_RING_CH,   0, angleToPulse(ringAngle));
    motorino.setPWM(SERVO_WRIST_CH,  0, angleToPulse(wristAngle));

    Serial.print("INDEX:");  Serial.print(indexAngle);
    Serial.print(" MIDDLE:"); Serial.print(middleAngle);
    Serial.print(" RING:");   Serial.print(ringAngle);
    Serial.print(" WRIST:");  Serial.println(wristAngle);
  }
  delay(1);
}
