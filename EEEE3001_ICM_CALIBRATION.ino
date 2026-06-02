#include <Wire.h>
#include <Adafruit_ICM20948.h>

#define TCAADDR 0x70
#define IMU_PORT 2

Adafruit_ICM20948 icm;
float minX = 999.0, maxX = -999.0;
float minY = 999.0, maxY = -999.0;

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  
  tcaSelect(IMU_PORT);
  if (!icm.begin_I2C()) {
    Serial.println("IMU not found!");
    while(true);
  }
  Serial.println("Rotate the IMU completely in a flat circle now...");
}

void loop() {
  tcaSelect(IMU_PORT);
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  if (mag.magnetic.x < minX) minX = mag.magnetic.x;
  if (mag.magnetic.x > maxX) maxX = mag.magnetic.x;
  if (mag.magnetic.y < minY) minY = mag.magnetic.y;
  if (mag.magnetic.y > maxY) maxY = mag.magnetic.y;

  Serial.printf("Copy these values into your main code:\n");
  Serial.printf("  float hardIron_X_min = %.2f;\n", minX);
  Serial.printf("  float hardIron_X_max = %.2f;\n", maxX);
  Serial.printf("  float hardIron_Y_min = %.2f;\n", minY);
  Serial.printf("  float hardIron_Y_max = %.2f;\n\n", maxY);
  
  delay(200);
}
