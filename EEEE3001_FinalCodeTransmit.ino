/* 
 * Transmitter (Glove Side)
 */

#include "Arduino.h"
#include "ads.h"
#include <Wire.h>
#include <WiFi.h>
#include "ESP32_NOW.h"
#include <esp_mac.h>
#include <Adafruit_ICM20948.h>

#define ADS_RESET_PIN 25   
#define TIMING_PIN 26
#define ESPNOW_PIN 27       
#define TCAADDR 0x70

#define IMU_PORT 2
#define FLEX_INDEX_PORT 7
#define FLEX_MIDDLE_PORT 5
#define FLEX_RING_PORT 3

#define ESPNOW_WIFI_CHANNEL 6

/* ESP-NOW Struct Data Types (Restored to Arrays) */
typedef struct {
  float bend[3];     // Index 0=Thumb, 1=Index, 2=Middle
  float stretch[3];  // Index 0=Thumb, 1=Index, 2=Middle
  float heading;     // IMU orientation calculation
} gloveData;

gloveData data;
Adafruit_ICM20948 icm;

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) 
    : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  ~ESP_NOW_Broadcast_Peer() { remove(); }

  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data_ptr, size_t len) {
    if (!send(data_ptr, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void ads_data_callback(float * sample);
void deadzone_filter(float * sample, uint8_t sensor_idx);
void signal_filter(float * sample, uint8_t sensor_idx);
void parse_com_port(void);
void init_flex_sensor(uint8_t port);
void ads_data_callback(float * sample, uint8_t sample_type) {}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  pinMode(TIMING_PIN, OUTPUT);
  digitalWrite(TIMING_PIN, LOW);

  pinMode(ESPNOW_PIN, OUTPUT);
  digitalWrite(ESPNOW_PIN, LOW);

  tcaSelect(IMU_PORT);
  delay(10);
  if (!icm.begin_I2C()) {
    Serial.println("IMU not found on Port 2!");
    while(true);
  }
  Serial.println("IMU initialized successfully.");

  Serial.println("Initializing Flex Sensors...");
  init_flex_sensor(FLEX_INDEX_PORT);
  init_flex_sensor(FLEX_MIDDLE_PORT);
  init_flex_sensor(FLEX_RING_PORT);
  Serial.println("All Flex Sensors initialized successfully.");

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer!");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Setup complete. Streaming data...");
}

void loop() {
  uint8_t data_type;
  float temp_sample[2]; // Fixed: Needs array length 2 for bend + stretch metrics

  // --- READ INDEX (Port 7) ---
  tcaSelect(FLEX_INDEX_PORT);
  if (ads_read_polled(temp_sample, &data_type) == ADS_OK && data_type == ADS_SAMPLE) {
    signal_filter(temp_sample, 0);
    deadzone_filter(temp_sample, 0);
    data.bend[0]    = temp_sample[0];
    data.stretch[0] = temp_sample[1];
  }

  // --- READ MIDDLE (Port 6) ---
  tcaSelect(FLEX_MIDDLE_PORT);
  if (ads_read_polled(temp_sample, &data_type) == ADS_OK && data_type == ADS_SAMPLE) {
    signal_filter(temp_sample, 1);
    deadzone_filter(temp_sample, 1);
    data.bend[1]    = temp_sample[0];
    data.stretch[1] = temp_sample[1];
  }

  // --- READ RING (Port 5) ---
  tcaSelect(FLEX_RING_PORT);
  if (ads_read_polled(temp_sample, &data_type) == ADS_OK && data_type == ADS_SAMPLE) {
    signal_filter(temp_sample, 2);
    deadzone_filter(temp_sample, 2);
    data.bend[2]    = temp_sample[0];
    data.stretch[2] = temp_sample[1];
  }

  // --- READ IMU (Port 2) ---
  digitalWrite(TIMING_PIN, HIGH);  // Start timing
  tcaSelect(IMU_PORT);
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  digitalWrite(TIMING_PIN, LOW);   // Stop timing

  // Calibration Values
  float hardIron_X_min = -3.75;
  float hardIron_X_max = 76.65;
  float hardIron_Y_min = -30.15;
  float hardIron_Y_max = 55.50;

  float offsetX = (hardIron_X_max + hardIron_X_min) / 2.0; 
  float offsetY = (hardIron_Y_max + hardIron_Y_min) / 2.0; 

  float rangeX = (hardIron_X_max - hardIron_X_min) / 2.0;
  float rangeY = (hardIron_Y_max - hardIron_Y_min) / 2.0;
  float avgRange = (rangeX + rangeY) / 2.0;

  float scaleX = avgRange / rangeX;
  float scaleY = avgRange / rangeY;

  float correctedX = (mag.magnetic.x - offsetX) * scaleX;
  float correctedY = (mag.magnetic.y - offsetY) * scaleY;

  float heading_rad = atan2(correctedY, correctedX);
  data.heading = heading_rad * 180.0 / PI;

  if (data.heading < 0) { data.heading += 360.0; }

  // --- PRINT DATA TO SERIAL ---
  Serial.print("INDEX_Bend:");  Serial.print(data.bend[0]);   Serial.print(" ");
  Serial.print("MIDDLE_Bend:");  Serial.print(data.bend[1]);   Serial.print(" ");
  Serial.print("RING_Bend:"); Serial.print(data.bend[2]);   Serial.print(" ");
  Serial.print("IMU_Heading:"); Serial.println(data.heading);

  // --- TRANSMIT DATA VIA ESP-NOW ---
  digitalWrite(ESPNOW_PIN, HIGH);
  broadcast_peer.send_message((uint8_t *)&data, sizeof(data));
  digitalWrite(ESPNOW_PIN, LOW);

  if(Serial.available()) { parse_com_port(); }
  delay(5);
}

void init_flex_sensor(uint8_t port) {
  tcaSelect(port);
  delay(10);
  ads_init_t init;
  init.sps = ADS_100_HZ;
  init.ads_sample_callback = &ads_data_callback;
  init.reset_pin = ADS_RESET_PIN;
  init.datardy_pin = -1;  
  init.addr = 0;

  if(ads_init(&init) != ADS_OK) {
    Serial.printf("Sensor on Port %d failed initialization\n", port);
  } else {
    ads_stretch_en(true);
    ads_polled(true);
  }
}

void parse_com_port(void) {
  char key = Serial.read();
  switch(key) {
    case '0': ads_calibrate(ADS_CALIBRATE_FIRST, 0); break;
    case '9': ads_calibrate(ADS_CALIBRATE_SECOND, 90); break;
    case 'c': ads_calibrate(ADS_CALIBRATE_CLEAR, 0); break;
    case 'r': ads_run(true); break;
    case 's': ads_run(false); break;
    default: break;
  }
}

void signal_filter(float * sample, uint8_t sensor_idx) {
    static float filter_samples[3][2][6];
    for(uint8_t i=0; i<2; i++) {
      filter_samples[sensor_idx][i][5] = filter_samples[sensor_idx][i][4];
      filter_samples[sensor_idx][i][4] = filter_samples[sensor_idx][i][3];
      filter_samples[sensor_idx][i][3] = (float)sample[i];
      filter_samples[sensor_idx][i][2] = filter_samples[sensor_idx][i][1];
      filter_samples[sensor_idx][i][1] = filter_samples[sensor_idx][i][0];
      filter_samples[sensor_idx][i][0] = filter_samples[sensor_idx][i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[sensor_idx][i][2] + \
        0.20657208382614792f*(filter_samples[sensor_idx][i][3] + 2*filter_samples[sensor_idx][i][4] + filter_samples[sensor_idx][i][5]);   
      sample[i] = filter_samples[sensor_idx][i][0];
    }
}

void deadzone_filter(float * sample, uint8_t sensor_idx) {
  static float prev_sample[3][2];
  float dead_zone = 0.75f;
  for(uint8_t i=0; i<2; i++) {
    if(fabs(sample[i]-prev_sample[sensor_idx][i]) > dead_zone)
      prev_sample[sensor_idx][i] = sample[i];
    else
      sample[i] = prev_sample[sensor_idx][i];
  }
}
