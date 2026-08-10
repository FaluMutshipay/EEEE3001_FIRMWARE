#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// Safe mapping: A3=RX, A4=TX (Unused)
SoftwareSerial espSerial(A3, A4); 
Adafruit_PWMServoDriver motorino = Adafruit_PWMServoDriver(0x40);

int currentAngles[] = {90, 90, 90, 90, 90, 90}; 

// Advanced Parsing Memory Buffer Configuration
const byte numChars = 64;
char receivedChars[numChars];
bool newData = false;

void setup() {
  Serial.begin(115200);   // Laptop debug log
  
  pinMode(A3, INPUT); 
  espSerial.begin(9600);  
  
  motorino.begin();
  motorino.setOscillatorFrequency(27000000);
  motorino.setPWMFreq(50); 

  // Rigidly anchor arm to standard vertical standby posture on boot
  for (int i = 0; i <= 4; i++) motorino.writeMicroseconds(i, 1500);
  motorino.writeMicroseconds(5, 1600);
  Serial.println("Uno Parsing Protection Upgraded. Ready on A3...");
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    parseAndMoveArm();
  }
}

// Low-overhead background reader that builds packets safely character-by-character
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = 'A';
  char endMarker = '\n';
  char rc;
 
  while (espSerial.available() > 0 && newData == false) {
    rc = espSerial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) { ndx = numChars - 1; }
      } else {
        receivedChars[ndx] = '\0'; // Terminate the text memory block cleanly
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// Splits the text buffer by commas and pushes the integers straight to the PCA9685 chip
void parseAndMoveArm() {
  char *strtokIndx; 
  
  // Extract first integer (Base)
  strtokIndx = strtok(receivedChars, ",");
  if (strtokIndx != NULL) currentAngles[0] = atoi(strtokIndx);
  
  // Extract remaining 5 joints
  for (int i = 1; i < 6; i++) {
    strtokIndx = strtok(NULL, ",");
    if (strtokIndx != NULL) currentAngles[i] = atoi(strtokIndx);
  }

  // --- DRIVE THE PHYSICAL HARDWARE PORTS ---
  // Map and update main frame joints (Channels 0 to 4)
  for (int channel = 0; channel <= 4; channel++) {
    int cleanAngle = constrain(currentAngles[channel], 0, 180);
    int microSecs = map(cleanAngle, 0, 180, 500, 2500);
    motorino.writeMicroseconds(channel, microSecs);
  }
  
  // Map and update Grip claw execution (Channel 5)
  int clawAngle = constrain(currentAngles[5], 0, 180);
  int clawMicroSecs = map(clawAngle, 0, 180, 500, 2500);
  motorino.writeMicroseconds(5, clawMicroSecs);

  // FIXED: Standard Arduino printing style replacing .printf()
  Serial.print("ARM MOVED -> B:");   Serial.print(currentAngles[0]);
  Serial.print(" | S:");           Serial.print(currentAngles[1]);
  Serial.print(" | E:");           Serial.print(currentAngles[2]);
  Serial.print(" | C:");           Serial.println(currentAngles[5]);

  newData = false; // Reset processing flag for the next wave
}
