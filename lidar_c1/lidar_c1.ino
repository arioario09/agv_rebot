// Copyright 2023-2025 KAIA.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESP32
  #error This example runs on ESP32
#endif

// PIN DEFINITION untuk RPLIDAR C1 (hbut TX/RX saja)
const uint8_t LIDAR_GPIO_RX = 16;   // ESP32 RX -> Lidar TX
const uint8_t LIDAR_GPIO_TX = 17;   // ESP32 TX -> Lidar RX

// LIDAR MODEL
#define SLAMTEC_RPLIDAR_C1

const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint16_t PRINT_EVERY_NTH_POINT = 10;

#include "lds_all_models.h"

HardwareSerial LidarSerial(2);
LDS *lidar;

void setupLidar() {
  lidar = new LDS_RPLIDAR_C1();

  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);

  // Setup serial lidar - HANYA serial communication
  LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_RX, LIDAR_GPIO_TX);
  LidarSerial.setTimeout(100);

  delay(1000);
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  delay(2000);  // Tunggu serial monitor

  Serial.println();
  Serial.println("=== RPLIDAR C1 WITH INTERNAL MOTOR CONTROL ===");
  Serial.println("Motor will start automatically when power is applied");
  Serial.println("ESP32 only handles serial communication");
  
  setupLidar();

  Serial.print("Model: ");
  Serial.println(lidar->getModelName());
  Serial.print("Baud Rate: ");
  Serial.println(lidar->getSerialBaudRate());
  Serial.print("TX Pin: GPIO ");
  Serial.println(LIDAR_GPIO_TX);
  Serial.print("RX Pin: GPIO ");
  Serial.println(LIDAR_GPIO_RX);

  // Start lidar scanning
  Serial.println("\nStarting lidar scanning...");
  LDS::result_t result = lidar->start();
  
  Serial.print("Start result: ");
  Serial.println(lidar->resultCodeToString(result));

  if (result != LDS::RESULT_OK) {
    Serial.println("Trying to stop and restart...");
    lidar->stop();
    delay(1000);
    result = lidar->start();
    Serial.print("Restart result: ");
    Serial.println(lidar->resultCodeToString(result));
  }
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  static int pointCount = 0;
  static int scanCount = 0;
  
  if (scan_completed) {
    scanCount++;
    float scanFreq = lidar->getCurrentScanFreqHz();
    Serial.println("=========================================");
    Serial.print("SCAN #");
    Serial.print(scanCount);
    Serial.print(" COMPLETED");
    Serial.print(" | Points: ");
    Serial.print(pointCount);
    Serial.print(" | Scan Frequency: ");
    Serial.print(scanFreq);
    Serial.println(" Hz");
    Serial.println("=========================================");
    
    pointCount = 0;
    return;
  }

  pointCount++;
  
  // Filter data yang valid saja
  if (distance_mm > 10.0 && distance_mm < 10000.0 && angle_deg >= 0.0 && angle_deg <= 360.0) {
    if (pointCount % PRINT_EVERY_NTH_POINT == 0) {
      Serial.print("[");
      Serial.print(pointCount);
      Serial.print("] Angle: ");
      Serial.print(angle_deg, 1);
      Serial.print("° | Distance: ");
      Serial.print(distance_mm, 1);
      Serial.print("mm | Quality: ");
      Serial.println(quality, 1);
    }
  } else {
    // Data invalid - tampilkan sebagai debug
    if (pointCount % 50 == 0) {
      Serial.print("[INVALID] Angle: ");
      Serial.print(angle_deg, 1);
      Serial.print("° | Distance: ");
      Serial.print(distance_mm, 1);
      Serial.print("mm | Quality: ");
      Serial.println(quality, 1);
    }
  }
}

// Callback function untuk serial communication
int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("[INFO] ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("[ERROR] ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  // Optional: bisa diaktifkan untuk debug packet
  // Serial.print("[PACKET] Length: ");
  // Serial.println(length);
}

void loop() {
  lidar->loop();
  
  // Status report periodic
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 10000) { // Setiap 10 detik
    lastStatus = millis();
    float scanFreq = lidar->getCurrentScanFreqHz();
    Serial.print("[STATUS] Scan Frequency: ");
    Serial.print(scanFreq);
    Serial.println(" Hz");
    
    if (scanFreq < 1.0) {
      Serial.println("[WARNING] Low scan frequency - check motor rotation!");
    }
  }
  delay(10);
}