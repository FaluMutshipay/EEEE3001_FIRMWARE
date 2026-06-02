#include <Wire.h>

void tcaSelect(uint8_t i) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200); // Set Serial Monitor to 115200
  Wire.begin();
  Serial.println("Starting Mux Scan...");
}

void loop() {
  // 1. Check the Main Bus (IMU should be here)
  Serial.println("--- Checking Main Bus ---");
  
  // Close all mux ports first to see only the main bus
  Wire.beginTransmission(0x70);
  Wire.write(0); 
  Wire.endTransmission();

  scanBus(); 

  // 2. Check each Port individually
  for (uint8_t port = 0; port < 8; port++) {
    Serial.print("--- Checking Port SD"); Serial.print(port); Serial.println(" ---");
    tcaSelect(port);
    scanBus();
  }

  Serial.println("Scan complete. Restarting in 10s...");
  delay(10000);
}

// Helper function to scan whatever is currently "visible"
void scanBus() {
  bool found = false;
  for (uint8_t addr = 1; addr <= 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      found = true;
    }
  }
  if (!found) Serial.println("No devices found on this path.");
}
