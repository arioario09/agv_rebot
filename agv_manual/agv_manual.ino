// Copyright 2023-2025 KAIA.AI
// Modified for Omniwheel + RPLIDAR C1 + Collision Avoidance

#ifndef ESP32
  #error This example runs on ESP32
#endif

#include <AccelStepper.h>

// ========== PIN DEFINITIONS MOTOR STEPPER ==========
// Motor 1
#define enablePin1 23
#define dirPin1 4
#define stepPin1 2

// Motor 2
#define enablePin2 19
#define dirPin2 18
#define stepPin2 32

// Motor 3
#define enablePin3 27
#define dirPin3 14
#define stepPin3 12

// Motor 4
#define enablePin4 33
#define dirPin4 25
#define stepPin4 26

// ========== PIN DEFINITIONS LIDAR ==========
const uint8_t LIDAR_GPIO_RX = 16;   // ESP32 RX -> Lidar TX
const uint8_t LIDAR_GPIO_TX = 17;   // ESP32 TX -> Lidar RX

// ========== LIDAR CONFIGURATION ==========
#define SLAMTEC_RPLIDAR_C1
const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint16_t PRINT_EVERY_NTH_POINT = 100;

#include "lds_all_models.h"

// ========== COLLISION AVOIDANCE SETTINGS ==========
const float DANGER_DISTANCE = 300.0;    // mm - jarak bahaya (berhenti total)
const float WARNING_DISTANCE = 500.0;   // mm - jarak peringatan (lambat)
const float SLOW_SPEED_FACTOR = 0.3;    // Kecepatan lambat = 30% dari normal

// Zona deteksi (dalam derajat)
const float ZONE_FRONT_MIN = 315.0;     // 315° - 45° = Depan
const float ZONE_FRONT_MAX = 45.0;
const float ZONE_BACK_MIN = 135.0;      // 135° - 225° = Belakang
const float ZONE_BACK_MAX = 225.0;
const float ZONE_LEFT_MIN = 45.0;       // 45° - 135° = Kiri
const float ZONE_LEFT_MAX = 135.0;
const float ZONE_RIGHT_MIN = 225.0;     // 225° - 315° = Kanan
const float ZONE_RIGHT_MAX = 315.0;

// ========== INISIALISASI MOTOR ==========
AccelStepper motor4(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor3(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper motor1(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper motor2(AccelStepper::DRIVER, stepPin4, dirPin4);

// Kecepatan motor (TB6600 dengan 1/8 microstep)
const int maxSpeed = 6400;
const int acceleration = 3200;

// ========== INISIALISASI LIDAR ==========
HardwareSerial LidarSerial(2);
LDS *lidar;

// ========== COLLISION DETECTION VARIABLES ==========
struct ObstacleZone {
  float minDistance;
  bool dangerDetected;
  bool warningDetected;
  unsigned long lastUpdate;
};

ObstacleZone zoneFront = {10000.0, false, false, 0};
ObstacleZone zoneBack = {10000.0, false, false, 0};
ObstacleZone zoneLeft = {10000.0, false, false, 0};
ObstacleZone zoneRight = {10000.0, false, false, 0};

// Movement state
enum MovementState {
  STOPPED,
  MOVING_FORWARD,
  MOVING_BACKWARD,
  STRAFING_LEFT,
  STRAFING_RIGHT,
  ROTATING_LEFT,
  ROTATING_RIGHT
};

MovementState currentMovement = STOPPED;
bool collisionAvoidanceEnabled = true;

// ========== SETUP ==========
void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  delay(2000);
  
  Serial.println();
  Serial.println("==========================================");
  Serial.println("  OMNIWHEEL + RPLIDAR C1 + ANTI-COLLISION");
  Serial.println("==========================================");
  
  // Setup Motor Stepper
  setupMotors();
  
  // Setup LIDAR
  setupLidar();
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Motor Commands:");
  Serial.println("  w=forward, s=backward, a=left, d=right");
  Serial.println("  q=rotate-left, e=rotate-right, x=stop");
  Serial.println("Safety Commands:");
  Serial.println("  c=toggle collision avoidance ON/OFF");
  Serial.println("\nCollision Avoidance: ENABLED");
  Serial.println("Danger Distance: " + String(DANGER_DISTANCE) + "mm");
  Serial.println("Warning Distance: " + String(WARNING_DISTANCE) + "mm");
  Serial.println("==========================================\n");
}

void setupMotors() {
  Serial.println("\n--- Setting up Motors ---");
  
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(enablePin3, OUTPUT);
  pinMode(enablePin4, OUTPUT);
  
  digitalWrite(enablePin1, LOW);
  digitalWrite(enablePin2, LOW);
  digitalWrite(enablePin3, LOW);
  digitalWrite(enablePin4, LOW);
  
  motor1.setMaxSpeed(maxSpeed);
  motor1.setAcceleration(acceleration);
  motor2.setMaxSpeed(maxSpeed);
  motor2.setAcceleration(acceleration);
  motor3.setMaxSpeed(maxSpeed);
  motor3.setAcceleration(acceleration);
  motor4.setMaxSpeed(maxSpeed);
  motor4.setAcceleration(acceleration);
  
  Serial.println("Motors initialized successfully");
}

void setupLidar() {
  Serial.println("\n--- Setting up LIDAR ---");
  
  lidar = new LDS_RPLIDAR_C1();
  
  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);
  
  LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_RX, LIDAR_GPIO_TX);
  LidarSerial.setTimeout(100);
  
  delay(1000);
  
  Serial.print("Model: ");
  Serial.println(lidar->getModelName());
  
  LDS::result_t result = lidar->start();
  Serial.print("Start result: ");
  Serial.println(lidar->resultCodeToString(result));
  
  if (result != LDS::RESULT_OK) {
    lidar->stop();
    delay(1000);
    result = lidar->start();
  }
}

// ========== MAIN LOOP ==========
void loop() {
  lidar->loop();
  
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command != '\n' && command != '\r') {
      handleMotorCommand(command);
    }
  }
  
  // Apply collision avoidance
  if (collisionAvoidanceEnabled) {
    checkAndApplyCollisionAvoidance();
  }
  
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
  
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    printStatus();
  }
}

// ========== COLLISION AVOIDANCE ==========
void checkAndApplyCollisionAvoidance() {
  // Reset deteksi jika data sudah lama (>500ms)
  unsigned long now = millis();
  if (now - zoneFront.lastUpdate > 500) zoneFront.dangerDetected = zoneFront.warningDetected = false;
  if (now - zoneBack.lastUpdate > 500) zoneBack.dangerDetected = zoneBack.warningDetected = false;
  if (now - zoneLeft.lastUpdate > 500) zoneLeft.dangerDetected = zoneLeft.warningDetected = false;
  if (now - zoneRight.lastUpdate > 500) zoneRight.dangerDetected = zoneRight.warningDetected = false;
  
  bool emergencyStop = false;
  float speedFactor = 1.0;
  
  // Cek bahaya berdasarkan arah gerakan
  switch (currentMovement) {
    case MOVING_FORWARD:
      if (zoneFront.dangerDetected) {
        emergencyStop = true;
        Serial.println("[SAFETY] EMERGENCY STOP! Obstacle ahead!");
      } else if (zoneFront.warningDetected) {
        speedFactor = SLOW_SPEED_FACTOR;
        Serial.println("[SAFETY] Slowing down - obstacle detected");
      }
      break;
      
    case MOVING_BACKWARD:
      if (zoneBack.dangerDetected) {
        emergencyStop = true;
        Serial.println("[SAFETY] EMERGENCY STOP! Obstacle behind!");
      } else if (zoneBack.warningDetected) {
        speedFactor = SLOW_SPEED_FACTOR;
      }
      break;
      
    case STRAFING_LEFT:
      if (zoneLeft.dangerDetected) {
        emergencyStop = true;
        Serial.println("[SAFETY] EMERGENCY STOP! Obstacle on left!");
      } else if (zoneLeft.warningDetected) {
        speedFactor = SLOW_SPEED_FACTOR;
      }
      break;
      
    case STRAFING_RIGHT:
      if (zoneRight.dangerDetected) {
        emergencyStop = true;
        Serial.println("[SAFETY] EMERGENCY STOP! Obstacle on right!");
      } else if (zoneRight.warningDetected) {
        speedFactor = SLOW_SPEED_FACTOR;
      }
      break;
      
    case ROTATING_LEFT:
    case ROTATING_RIGHT:
      // Cek semua zona saat rotasi
      if (zoneFront.dangerDetected || zoneBack.dangerDetected || 
          zoneLeft.dangerDetected || zoneRight.dangerDetected) {
        speedFactor = SLOW_SPEED_FACTOR;
      }
      break;
      
    case STOPPED:
      return;
  }
  
  if (emergencyStop) {
    stopAllMotors();
    currentMovement = STOPPED;
  } else if (speedFactor < 1.0) {
    adjustMotorSpeed(speedFactor);
  }
}

void adjustMotorSpeed(float factor) {
  int adjustedSpeed = maxSpeed * factor;
  
  switch (currentMovement) {
    case MOVING_FORWARD:
      motor1.setSpeed(adjustedSpeed);
      motor2.setSpeed(-adjustedSpeed);
      motor3.setSpeed(-adjustedSpeed);
      motor4.setSpeed(adjustedSpeed);
      break;
    case MOVING_BACKWARD:
      motor1.setSpeed(-adjustedSpeed);
      motor2.setSpeed(adjustedSpeed);
      motor3.setSpeed(adjustedSpeed);
      motor4.setSpeed(-adjustedSpeed);
      break;
    case STRAFING_LEFT:
      motor1.setSpeed(-adjustedSpeed);
      motor2.setSpeed(-adjustedSpeed);
      motor3.setSpeed(adjustedSpeed);
      motor4.setSpeed(adjustedSpeed);
      break;
    case STRAFING_RIGHT:
      motor1.setSpeed(adjustedSpeed);
      motor2.setSpeed(adjustedSpeed);
      motor3.setSpeed(-adjustedSpeed);
      motor4.setSpeed(-adjustedSpeed);
      break;
    case ROTATING_LEFT:
      motor1.setSpeed(-adjustedSpeed);
      motor2.setSpeed(-adjustedSpeed);
      motor3.setSpeed(-adjustedSpeed);
      motor4.setSpeed(-adjustedSpeed);
      break;
    case ROTATING_RIGHT:
      motor1.setSpeed(adjustedSpeed);
      motor2.setSpeed(adjustedSpeed);
      motor3.setSpeed(adjustedSpeed);
      motor4.setSpeed(adjustedSpeed);
      break;
    default:
      break;
  }
}

// ========== MOTOR CONTROL ==========
void handleMotorCommand(char cmd) {
  switch(cmd) {
    case 'w':
      currentMovement = MOVING_FORWARD;
      moveForward();
      Serial.println("[MOTOR] Moving Forward");
      break;
      
    case 's':
      currentMovement = MOVING_BACKWARD;
      moveBackward();
      Serial.println("[MOTOR] Moving Backward");
      break;
      
    case 'a':
      currentMovement = STRAFING_LEFT;
      strafeLeft();
      Serial.println("[MOTOR] Strafing Left");
      break;
      
    case 'd':
      currentMovement = STRAFING_RIGHT;
      strafeRight();
      Serial.println("[MOTOR] Strafing Right");
      break;
      
    case 'q':
      currentMovement = ROTATING_LEFT;
      rotateLeft();
      Serial.println("[MOTOR] Rotating Left");
      break;
      
    case 'e':
      currentMovement = ROTATING_RIGHT;
      rotateRight();
      Serial.println("[MOTOR] Rotating Right");
      break;
      
    case 'x':
      currentMovement = STOPPED;
      stopAllMotors();
      Serial.println("[MOTOR] Stop");
      break;
      
    case 'c':
      collisionAvoidanceEnabled = !collisionAvoidanceEnabled;
      Serial.print("[SAFETY] Collision Avoidance: ");
      Serial.println(collisionAvoidanceEnabled ? "ENABLED" : "DISABLED");
      break;
      
    default:
      break;
  }
}

void moveForward() {
  motor1.setSpeed(maxSpeed);
  motor2.setSpeed(-maxSpeed);
  motor3.setSpeed(-maxSpeed);
  motor4.setSpeed(maxSpeed);
}

void moveBackward() {
  motor1.setSpeed(-maxSpeed);
  motor2.setSpeed(maxSpeed);
  motor3.setSpeed(maxSpeed);
  motor4.setSpeed(-maxSpeed);
}

void strafeLeft() {
  motor1.setSpeed(-maxSpeed);
  motor2.setSpeed(-maxSpeed);
  motor3.setSpeed(maxSpeed);
  motor4.setSpeed(maxSpeed);
}

void strafeRight() {
  motor1.setSpeed(maxSpeed);
  motor2.setSpeed(maxSpeed);
  motor3.setSpeed(-maxSpeed);
  motor4.setSpeed(-maxSpeed);
}

void rotateLeft() {
  motor1.setSpeed(-maxSpeed);
  motor2.setSpeed(-maxSpeed);
  motor3.setSpeed(-maxSpeed);
  motor4.setSpeed(-maxSpeed);
}

void rotateRight() {
  motor1.setSpeed(maxSpeed);
  motor2.setSpeed(maxSpeed);
  motor3.setSpeed(maxSpeed);
  motor4.setSpeed(maxSpeed);
}

void stopAllMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
}

// ========== LIDAR CALLBACKS ==========
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  if (scan_completed) {
    // Reset min distances setiap scan baru
    zoneFront.minDistance = 10000.0;
    zoneBack.minDistance = 10000.0;
    zoneLeft.minDistance = 10000.0;
    zoneRight.minDistance = 10000.0;
    return;
  }
  
  // Filter data valid
  if (distance_mm < 10.0 || distance_mm > 10000.0) return;
  
  unsigned long now = millis();
  
  // Deteksi zona dan update jarak minimum
  if ((angle_deg >= ZONE_FRONT_MIN && angle_deg <= 360.0) || 
      (angle_deg >= 0.0 && angle_deg <= ZONE_FRONT_MAX)) {
    // Zona Depan
    if (distance_mm < zoneFront.minDistance) {
      zoneFront.minDistance = distance_mm;
      zoneFront.dangerDetected = (distance_mm < DANGER_DISTANCE);
      zoneFront.warningDetected = (distance_mm < WARNING_DISTANCE);
      zoneFront.lastUpdate = now;
    }
  } else if (angle_deg >= ZONE_BACK_MIN && angle_deg <= ZONE_BACK_MAX) {
    // Zona Belakang
    if (distance_mm < zoneBack.minDistance) {
      zoneBack.minDistance = distance_mm;
      zoneBack.dangerDetected = (distance_mm < DANGER_DISTANCE);
      zoneBack.warningDetected = (distance_mm < WARNING_DISTANCE);
      zoneBack.lastUpdate = now;
    }
  } else if (angle_deg >= ZONE_LEFT_MIN && angle_deg <= ZONE_LEFT_MAX) {
    // Zona Kiri
    if (distance_mm < zoneLeft.minDistance) {
      zoneLeft.minDistance = distance_mm;
      zoneLeft.dangerDetected = (distance_mm < DANGER_DISTANCE);
      zoneLeft.warningDetected = (distance_mm < WARNING_DISTANCE);
      zoneLeft.lastUpdate = now;
    }
  } else if (angle_deg >= ZONE_RIGHT_MIN && angle_deg <= ZONE_RIGHT_MAX) {
    // Zona Kanan
    if (distance_mm < zoneRight.minDistance) {
      zoneRight.minDistance = distance_mm;
      zoneRight.dangerDetected = (distance_mm < DANGER_DISTANCE);
      zoneRight.warningDetected = (distance_mm < WARNING_DISTANCE);
      zoneRight.lastUpdate = now;
    }
  }
}

int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("[LIDAR INFO] ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("[LIDAR ERROR] ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  // Optional
}

// ========== STATUS REPORT ==========
void printStatus() {
  Serial.println("\n========== SYSTEM STATUS ==========");
  Serial.print("LIDAR Freq: ");
  Serial.print(lidar->getCurrentScanFreqHz());
  Serial.println(" Hz");
  
  Serial.println("\nObstacle Distances:");
  Serial.print("  Front: ");
  Serial.print(zoneFront.minDistance, 0);
  Serial.print("mm ");
  if (zoneFront.dangerDetected) Serial.print("[DANGER!]");
  else if (zoneFront.warningDetected) Serial.print("[Warning]");
  Serial.println();
  
  Serial.print("  Back:  ");
  Serial.print(zoneBack.minDistance, 0);
  Serial.print("mm ");
  if (zoneBack.dangerDetected) Serial.print("[DANGER!]");
  else if (zoneBack.warningDetected) Serial.print("[Warning]");
  Serial.println();
  
  Serial.print("  Left:  ");
  Serial.print(zoneLeft.minDistance, 0);
  Serial.print("mm ");
  if (zoneLeft.dangerDetected) Serial.print("[DANGER!]");
  else if (zoneLeft.warningDetected) Serial.print("[Warning]");
  Serial.println();
  
  Serial.print("  Right: ");
  Serial.print(zoneRight.minDistance, 0);
  Serial.print("mm ");
  if (zoneRight.dangerDetected) Serial.print("[DANGER!]");
  else if (zoneRight.warningDetected) Serial.print("[Warning]");
  Serial.println();
  
  Serial.print("\nMovement: ");
  switch(currentMovement) {
    case STOPPED: Serial.println("STOPPED"); break;
    case MOVING_FORWARD: Serial.println("FORWARD"); break;
    case MOVING_BACKWARD: Serial.println("BACKWARD"); break;
    case STRAFING_LEFT: Serial.println("LEFT"); break;
    case STRAFING_RIGHT: Serial.println("RIGHT"); break;
    case ROTATING_LEFT: Serial.println("ROTATE LEFT"); break;
    case ROTATING_RIGHT: Serial.println("ROTATE RIGHT"); break;
  }
  
  Serial.print("Collision Avoidance: ");
  Serial.println(collisionAvoidanceEnabled ? "ENABLED" : "DISABLED");
  Serial.println("===================================\n");
}