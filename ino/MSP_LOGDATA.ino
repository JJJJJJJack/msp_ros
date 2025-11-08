/*
 * Betaflight MSP v1 Communication
 * Reads: ATTITUDE, RAW_IMU, WP #118
 * Sends: RAW_RC
 * 
 * Connect Betaflight FC TX to Arduino RX (e.g., pin D7)
 * Connect Betaflight FC RX to Arduino TX (e.g., pin D6)
 */

#define RX_PIN D7
#define TX_PIN D6

// Debug flag - set to false to disable serial printing
#define DEBUG false

// MSP Commands (v1)
#define MSP_ATTITUDE        108
#define MSP_RAW_IMU         102
#define MSP_WP              118
#define MSP_SET_RAW_RC      200

// MSP Protocol bytes
#define MSP_HEADER          '$'
#define MSP_VERSION         'M'
#define MSP_REQUEST         '<'
#define MSP_RESPONSE        '>'

// RC channels (8 channels, typical range 1000-2000)
uint16_t rcData[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};


// Response buffer
uint8_t responseBuffer[256];
uint8_t responseOffset = 0;

//EspSoftwareSerial::UART Serial1;

// Attitude data
struct {
  int16_t roll;      // decidegrees
  int16_t pitch;     // decidegrees
  int16_t yaw;       // degrees
} attitude;

// Raw IMU data
struct {
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
  int16_t magX;
  int16_t magY;
  int16_t magZ;
} rawIMU;

// Waypoint data
struct {
  uint8_t wpNo;
  uint8_t action;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  uint16_t p1;
  uint16_t p2;
  uint16_t p3;
  uint8_t flag;
} waypoint;


void setup() {
  Serial.begin(115200);
  // For hardware serial, use the official setup
  // https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  
  Serial.println("Betaflight MSP v1 Interface");
  Serial.println("Starting communication...");
  delay(1000);
}

void loop() {
  // Read data from FC
  requestAttitude();
  delay(20);
  
//  requestRawIMU();
//  delay(20);

  // Get debug information
//  requestWaypoint(118);
//  delay(20);
//  
//  // Send RC data to FC
//  sendRawRC();
//  delay(50);
  
  // Optional: Modify RC values here for testing
  // rcData[0] = 1600; // Roll
  // rcData[1] = 1500; // Pitch
  // rcData[2] = 1500; // Throttle
  // rcData[3] = 1500; // Yaw
}

// Request ATTITUDE
void requestAttitude() {
  sendMSPRequest(MSP_ATTITUDE, NULL, 0);
  if (waitForResponse(MSP_ATTITUDE, 6, 2000)) {
    attitude.roll = read16();
    attitude.pitch = read16();
    attitude.yaw = read16();
    
    if (true) {
      Serial.println("=== ATTITUDE ===");
      Serial.print("Roll: "); Serial.print(attitude.roll / 10.0);
      Serial.print("° Pitch: "); Serial.print(attitude.pitch / 10.0);
      Serial.print("° Yaw: "); Serial.print(attitude.yaw);
      Serial.println("°");
    }
  }
}

// Request RAW_IMU
void requestRawIMU() {
  sendMSPRequest(MSP_RAW_IMU, NULL, 0);
  if (waitForResponse(MSP_RAW_IMU, 18, 200)) {
    rawIMU.accX = read16();
    rawIMU.accY = read16();
    rawIMU.accZ = read16();
    rawIMU.gyrX = read16();
    rawIMU.gyrY = read16();
    rawIMU.gyrZ = read16();
    rawIMU.magX = read16();
    rawIMU.magY = read16();
    rawIMU.magZ = read16();
    
    if (true) {
      Serial.println("=== RAW IMU ===");
      Serial.print("ACC["); Serial.print(rawIMU.accX);
      Serial.print(","); Serial.print(rawIMU.accY);
      Serial.print(","); Serial.print(rawIMU.accZ); Serial.print("] ");
      Serial.print("GYR["); Serial.print(rawIMU.gyrX);
      Serial.print(","); Serial.print(rawIMU.gyrY);
      Serial.print(","); Serial.print(rawIMU.gyrZ); Serial.print("] ");
      Serial.print("MAG["); Serial.print(rawIMU.magX);
      Serial.print(","); Serial.print(rawIMU.magY);
      Serial.print(","); Serial.print(rawIMU.magZ); Serial.println("]");
    }
  }
}

// Request Waypoint
void requestWaypoint(uint8_t wpNumber) {
  uint8_t payload[1] = {wpNumber};
  sendMSPRequest(MSP_WP, payload, 1);
  if (waitForResponse(MSP_WP, 21, 200)) {
    waypoint.wpNo = read8();
    waypoint.action = read8();
    waypoint.lat = read32();
    waypoint.lon = read32();
    waypoint.alt = read32();
    waypoint.p1 = read16();
    waypoint.p2 = read16();
    waypoint.p3 = read16();
    waypoint.flag = read8();
    
    if (DEBUG) {
      Serial.println("=== WAYPOINT 118 ===");
      Serial.print("WP#: "); Serial.print(waypoint.wpNo);
      Serial.print(" Action: "); Serial.println(waypoint.action);
      Serial.print("Lat: "); Serial.print(waypoint.lat / 10000000.0, 7);
      Serial.print(" Lon: "); Serial.print(waypoint.lon / 10000000.0, 7);
      Serial.print(" Alt: "); Serial.print(waypoint.alt / 100.0);
      Serial.println(" m");
      Serial.print("P1: "); Serial.print(waypoint.p1);
      Serial.print(" P2: "); Serial.print(waypoint.p2);
      Serial.print(" P3: "); Serial.print(waypoint.p3);
      Serial.print(" Flag: "); Serial.println(waypoint.flag);
    }
  }
}

// Send RAW_RC
void sendRawRC() {
  uint8_t payload[16];
  for (int i = 0; i < 8; i++) {
    payload[i * 2] = rcData[i] & 0xFF;
    payload[i * 2 + 1] = (rcData[i] >> 8) & 0xFF;
  }
  
  sendMSPRequest(MSP_SET_RAW_RC, payload, 16);
  
  if (DEBUG) {
    Serial.println("=== RAW RC SENT ===");
    Serial.print("RC: [");
    for (int i = 0; i < 8; i++) {
      Serial.print(rcData[i]);
      if (i < 7) Serial.print(",");
    }
    Serial.println("]");
  }
}

// Send MSP request
void sendMSPRequest(uint8_t cmd, uint8_t *data, uint8_t dataSize) {
  uint8_t checksum = 0;
  
  if (DEBUG) {
    Serial.print(">> SEND: $M< ");
    Serial.print("Size:"); Serial.print(dataSize);
    Serial.print(" Cmd:"); Serial.print(cmd);
    if (dataSize > 0) {
      Serial.print(" Data:[");
      for (uint8_t i = 0; i < dataSize; i++) {
        Serial.print(data[i], HEX);
        if (i < dataSize - 1) Serial.print(",");
      }
      Serial.print("]");
    }
  }
  
  Serial1.write(MSP_HEADER);
  Serial1.write(MSP_VERSION);
  Serial1.write(MSP_REQUEST);
  
  Serial1.write(dataSize);
  checksum ^= dataSize;
  
  Serial1.write(cmd);
  checksum ^= cmd;
  
  for (uint8_t i = 0; i < dataSize; i++) {
    Serial1.write(data[i]);
    checksum ^= data[i];
  }
  
  Serial1.write(checksum);
  
  if (DEBUG) {
    Serial.print(" Checksum:"); Serial.println(checksum, HEX);
  }
}

// Wait for MSP response
bool waitForResponse(uint8_t expectedCmd, uint8_t expectedSize, unsigned long timeout) {
  unsigned long startTime = millis();
  uint8_t state = 0;
  uint8_t dataSize = 0;
  uint8_t cmd = 0;
  uint8_t checksum = 0;
  uint8_t offset = 0;
  uint8_t buffer[256];
  
  while (millis() - startTime < timeout) {
    if (Serial1.available()) {
      uint8_t c = Serial1.read();
      switch (state) {
        case 0: // Wait for '$'
          if (c == MSP_HEADER) state = 1;
          break;
        case 1: // Wait for 'M'
          if (c == MSP_VERSION) state = 2;
          else state = 0;
          break;
        case 2: // Wait for '>'
          if (c == MSP_RESPONSE) state = 3;
          else state = 0;
          break;
        case 3: // Data size
          dataSize = c;
          checksum = c;
          state = 4;
          break;
        case 4: // Command
          cmd = c;
          checksum ^= c;
          state = 5;
          offset = 0;
          break;
        case 5: // Data
          buffer[offset++] = c;
          checksum ^= c;
          if (offset >= dataSize) state = 6;
          break;
        case 6: // Checksum
          if (checksum == c && cmd == expectedCmd) {
            memcpy(responseBuffer, buffer, dataSize);
            responseOffset = 0;
            return true;
          }
          state = 0;
          break;
      }
    }
  }
  
  if (DEBUG) {
    Serial.print("Timeout waiting for MSP ");
    Serial.println(expectedCmd);
  }
  return false;
}


// Read functions
uint8_t read8() {
  return responseBuffer[responseOffset++];
}

int16_t read16() {
  int16_t val = responseBuffer[responseOffset] | (responseBuffer[responseOffset + 1] << 8);
  responseOffset += 2;
  return val;
}

int32_t read32() {
  int32_t val = responseBuffer[responseOffset] | 
                (responseBuffer[responseOffset + 1] << 8) |
                (responseBuffer[responseOffset + 2] << 16) |
                (responseBuffer[responseOffset + 3] << 24);
  responseOffset += 4;
  return val;
}
