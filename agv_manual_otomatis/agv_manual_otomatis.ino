// Copyright 2023-2025 KAIA.AI
// Autonomous AGV with LIDAR Navigation & Obstacle Avoidance

#ifndef ESP32
  #error This example runs on ESP32
#endif

#include <AccelStepper.h>

// ========== PIN DEFINITIONS MOTOR STEPPER ==========
#define enablePin1 23
#define dirPin1 4
#define stepPin1 2
#define enablePin2 19
#define dirPin2 18
#define stepPin2 32
#define enablePin3 27
#define dirPin3 14
#define stepPin3 12
#define enablePin4 33
#define dirPin4 25
#define stepPin4 26

// ========== PIN DEFINITIONS LIDAR ==========
const uint8_t LIDAR_GPIO_RX = 16;
const uint8_t LIDAR_GPIO_TX = 17;

// ========== LIDAR CONFIGURATION ==========
#define SLAMTEC_RPLIDAR_C1
const uint32_t SERIAL_MONITOR_BAUD = 115200;

#include "lds_all_models.h"

// ========== NAVIGATION SETTINGS ==========
const float DANGER_DISTANCE = 400.0;      // mm - berhenti
const float WARNING_DISTANCE = 700.0;     // mm - mulai hindari
const float SAFE_DISTANCE = 1000.0;       // mm - aman untuk maju
const float SLOW_SPEED_FACTOR = 0.4;

// Zona deteksi yang lebih detail
const float ZONE_FRONT_LEFT_MIN = 330.0;
const float ZONE_FRONT_LEFT_MAX = 30.0;
const float ZONE_FRONT_RIGHT_MIN = 30.0;
const float ZONE_FRONT_RIGHT_MAX = 90.0;
const float ZONE_LEFT_MIN = 90.0;
const float ZONE_LEFT_MAX = 180.0;
const float ZONE_BACK_MIN = 180.0;
const float ZONE_BACK_MAX = 270.0;
const float ZONE_RIGHT_MIN = 270.0;
const float ZONE_RIGHT_MAX = 330.0;

// ========== AUTONOMOUS MODE SETTINGS ==========
const unsigned long STUCK_TIMEOUT = 5000;     // 5 detik stuck = ubah strategi
const unsigned long ROTATION_DURATION = 2000; // Durasi rotasi saat menghindari
const float MIN_CLEAR_ANGLE = 60.0;           // Minimal sudut bebas untuk maju

// ========== INISIALISASI MOTOR ==========
AccelStepper motor4(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor3(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper motor1(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper motor2(AccelStepper::DRIVER, stepPin4, dirPin4);

const int maxSpeed = 6400;
const int acceleration = 3200;

// ========== INISIALISASI LIDAR ==========
HardwareSerial LidarSerial(2);
LDS *lidar;

// ========== OBSTACLE DETECTION ==========
struct ObstacleZone {
  float minDistance;
  bool dangerDetected;
  bool warningDetected;
  unsigned long lastUpdate;
};

ObstacleZone zoneFrontLeft = {10000.0, false, false, 0};
ObstacleZone zoneFrontRight = {10000.0, false, false, 0};
ObstacleZone zoneLeft = {10000.0, false, false, 0};
ObstacleZone zoneRight = {10000.0, false, false, 0};
ObstacleZone zoneBack = {10000.0, false, false, 0};

// ========== AUTONOMOUS STATE MACHINE ==========
enum AutoMode {
  AUTO_DISABLED,
  AUTO_EXPLORING,
  AUTO_AVOIDING_OBSTACLE,
  AUTO_STUCK,
  AUTO_ROTATING_TO_CLEAR,
  AUTO_BACKING_UP
};

AutoMode autonomousMode = AUTO_DISABLED;
unsigned long lastMovementChange = 0;
unsigned long stuckTimer = 0;
bool wasMovingForward = false;
int consecutiveObstacles = 0;

// Direction preferences (untuk navigasi)
float bestClearAngle = 0.0;
float bestClearDistance = 0.0;

// ========== SETUP ==========
void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  delay(2000);
  
  Serial.println();
  Serial.println("============================================");
  Serial.println("  AUTONOMOUS AGV - LIDAR NAVIGATION SYSTEM");
  Serial.println("============================================");
  
  setupMotors();
  setupLidar();
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("\nControl Commands:");
  Serial.println("  AUTO MODE:");
  Serial.println("    1 = Start Autonomous Mode");
  Serial.println("    0 = Stop Autonomous Mode");
  Serial.println("\n  MANUAL MODE:");
  Serial.println("    w=forward, s=backward, a=left, d=right");
  Serial.println("    q=rotate-left, e=rotate-right, x=stop");
  Serial.println("\nSafety Settings:");
  Serial.println("  Danger: " + String(DANGER_DISTANCE) + "mm");
  Serial.println("  Warning: " + String(WARNING_DISTANCE) + "mm");
  Serial.println("  Safe: " + String(SAFE_DISTANCE) + "mm");
  Serial.println("============================================\n");
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
  
  Serial.println("Motors initialized");
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
  if (result != LDS::RESULT_OK) {
    lidar->stop();
    delay(1000);
    result = lidar->start();
  }
  Serial.println("LIDAR initialized");
}

// ========== MAIN LOOP ==========
void loop() {
  lidar->loop();
  
  // Handle serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command != '\n' && command != '\r') {
      handleCommand(command);
    }
  }
  
  // Run autonomous navigation
  if (autonomousMode != AUTO_DISABLED) {
    runAutonomousNavigation();
  }
  
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
  
  // Status report
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 3000) {
    lastStatus = millis();
    printStatus();
  }
}

// ========== AUTONOMOUS NAVIGATION ==========
void runAutonomousNavigation() {
  unsigned long now = millis();
  
  // Update zone status
  updateZoneStatus();
  
  switch (autonomousMode) {
    case AUTO_EXPLORING:
      // Mode eksplorasi normal
      if (zoneFrontLeft.dangerDetected || zoneFrontRight.dangerDetected) {
        // Obstacle detected!
        Serial.println("[AUTO] Obstacle detected! Switching to avoidance mode");
        autonomousMode = AUTO_AVOIDING_OBSTACLE;
        consecutiveObstacles++;
        stuckTimer = now;
        stopAllMotors();
        delay(200);
      } else if (zoneFrontLeft.warningDetected || zoneFrontRight.warningDetected) {
        // Warning zone - slow down
        moveForwardSlow();
      } else {
        // Clear path - full speed ahead
        if (!wasMovingForward) {
          Serial.println("[AUTO] Path clear - moving forward");
          wasMovingForward = true;
        }
        moveForward();
        consecutiveObstacles = 0;
        stuckTimer = now;
      }
      break;
      
    case AUTO_AVOIDING_OBSTACLE:
      // Find best direction
      findBestClearDirection();
      
      if (bestClearDistance > SAFE_DISTANCE) {
        // Found clear path
        Serial.print("[AUTO] Clear path found at ");
        Serial.print(bestClearAngle);
        Serial.println("°");
        autonomousMode = AUTO_ROTATING_TO_CLEAR;
        lastMovementChange = now;
      } else if (now - stuckTimer > STUCK_TIMEOUT) {
        // Stuck too long
        Serial.println("[AUTO] Stuck! Backing up");
        autonomousMode = AUTO_BACKING_UP;
        lastMovementChange = now;
      } else {
        // Keep rotating to find clear path
        if (bestClearAngle > 180.0) {
          rotateRight();
        } else {
          rotateLeft();
        }
      }
      break;
      
    case AUTO_ROTATING_TO_CLEAR:
      // Rotate menuju arah yang clear
      if (now - lastMovementChange < ROTATION_DURATION) {
        if (bestClearAngle > 180.0) {
          rotateRight();
        } else {
          rotateLeft();
        }
      } else {
        Serial.println("[AUTO] Rotation complete - resuming exploration");
        autonomousMode = AUTO_EXPLORING;
        wasMovingForward = false;
      }
      break;
      
    case AUTO_BACKING_UP:
      if (now - lastMovementChange < 2000) {
        // Mundur selama 2 detik
        moveBackward();
      } else {
        Serial.println("[AUTO] Backup complete - trying new direction");
        autonomousMode = AUTO_ROTATING_TO_CLEAR;
        lastMovementChange = now;
        // Rotate 90 derajat
        findBestClearDirection();
      }
      break;
      
    default:
      break;
  }
}

void findBestClearDirection() {
  // Cari arah dengan jarak terbebas
  bestClearDistance = 0.0;
  bestClearAngle = 0.0;
  
  // Check all zones
  if (zoneRight.minDistance > bestClearDistance) {
    bestClearDistance = zoneRight.minDistance;
    bestClearAngle = 300.0; // Kanan
  }
  if (zoneLeft.minDistance > bestClearDistance) {
    bestClearDistance = zoneLeft.minDistance;
    bestClearAngle = 90.0; // Kiri
  }
  if (zoneBack.minDistance > bestClearDistance && consecutiveObstacles > 2) {
    bestClearDistance = zoneBack.minDistance;
    bestClearAngle = 180.0; // Belakang (jika sudah stuck berulang)
  }
}

void updateZoneStatus() {
  unsigned long now = millis();
  if (now - zoneFrontLeft.lastUpdate > 500) {
    zoneFrontLeft.dangerDetected = zoneFrontLeft.warningDetected = false;
  }
  if (now - zoneFrontRight.lastUpdate > 500) {
    zoneFrontRight.dangerDetected = zoneFrontRight.warningDetected = false;
  }
  if (now - zoneLeft.lastUpdate > 500) {
    zoneLeft.dangerDetected = zoneLeft.warningDetected = false;
  }
  if (now - zoneRight.lastUpdate > 500) {
    zoneRight.dangerDetected = zoneRight.warningDetected = false;
  }
  if (now - zoneBack.lastUpdate > 500) {
    zoneBack.dangerDetected = zoneBack.warningDetected = false;
  }
}

// ========== COMMAND HANDLER ==========
void handleCommand(char cmd) {
  switch(cmd) {
    case '1':
      autonomousMode = AUTO_EXPLORING;
      consecutiveObstacles = 0;
      Serial.println("\n[AUTO] AUTONOMOUS MODE ENABLED");
      Serial.println("AGV will now navigate automatically");
      Serial.println("Press '0' to stop\n");
      break;
      
    case '0':
      autonomousMode = AUTO_DISABLED;
      stopAllMotors();
      Serial.println("\n[AUTO] AUTONOMOUS MODE DISABLED");
      Serial.println("Manual control enabled\n");
      break;
      
    case 'w':
      if (autonomousMode == AUTO_DISABLED) {
        moveForward();
        Serial.println("[MANUAL] Forward");
      }
      break;
      
    case 's':
      if (autonomousMode == AUTO_DISABLED) {
        moveBackward();
        Serial.println("[MANUAL] Backward");
      }
      break;
      
    case 'a':
      if (autonomousMode == AUTO_DISABLED) {
        strafeLeft();
        Serial.println("[MANUAL] Left");
      }
      break;
      
    case 'd':
      if (autonomousMode == AUTO_DISABLED) {
        strafeRight();
        Serial.println("[MANUAL] Right");
      }
      break;
      
    case 'q':
      if (autonomousMode == AUTO_DISABLED) {
        rotateLeft();
        Serial.println("[MANUAL] Rotate Left");
      }
      break;
      
    case 'e':
      if (autonomousMode == AUTO_DISABLED) {
        rotateRight();
        Serial.println("[MANUAL] Rotate Right");
      }
      break;
      
    case 'x':
      autonomousMode = AUTO_DISABLED;
      stopAllMotors();
      Serial.println("[MANUAL] Stop");
      break;
  }
}

// ========== MOTOR CONTROL ==========
void moveForward() {
  motor1.setSpeed(maxSpeed);
  motor2.setSpeed(-maxSpeed);
  motor3.setSpeed(-maxSpeed);
  motor4.setSpeed(maxSpeed);
}

void moveForwardSlow() {
  int slowSpeed = maxSpeed * SLOW_SPEED_FACTOR;
  motor1.setSpeed(slowSpeed);
  motor2.setSpeed(-slowSpeed);
  motor3.setSpeed(-slowSpeed);
  motor4.setSpeed(slowSpeed);
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
  int rotSpeed = maxSpeed * 0.6;
  motor1.setSpeed(-rotSpeed);
  motor2.setSpeed(-rotSpeed);
  motor3.setSpeed(-rotSpeed);
  motor4.setSpeed(-rotSpeed);
}

void rotateRight() {
  int rotSpeed = maxSpeed * 0.6;
  motor1.setSpeed(rotSpeed);
  motor2.setSpeed(rotSpeed);
  motor3.setSpeed(rotSpeed);
  motor4.setSpeed(rotSpeed);
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
  wasMovingForward = false;
}

// ========== LIDAR CALLBACKS ==========
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  if (scan_completed) {
    zoneFrontLeft.minDistance = 10000.0;
    zoneFrontRight.minDistance = 10000.0;
    zoneLeft.minDistance = 10000.0;
    zoneRight.minDistance = 10000.0;
    zoneBack.minDistance = 10000.0;
    return;
  }
  
  if (distance_mm < 10.0 || distance_mm > 10000.0) return;
  
  unsigned long now = millis();
  
  // Front Left (330-30°)
  if ((angle_deg >= ZONE_FRONT_LEFT_MIN && angle_deg <= 360.0) || 
      (angle_deg >= 0.0 && angle_deg <= ZONE_FRONT_LEFT_MAX)) {
    if (distance_mm < zoneFrontLeft.minDistance) {
      zoneFrontLeft.minDistance = distance_mm;
      zoneFrontLeft.dangerDetected = (distance_mm < DANGER_DISTANCE);
      zoneFrontLeft.warningDetected = (distance_mm < WARNING_DISTANCE);
      zoneFrontLeft.lastUpdate = now;
    }
  }
  // Front Right (30-90°)
  else if (angle_deg >= ZONE_FRONT_RIGHT_MIN && angle_deg <= ZONE_FRONT_RIGHT_MAX) {
    if (distance_mm < zoneFrontRight.minDistance) {
      zoneFrontRight.minDistance = distance_mm;
      zoneFrontRight.dangerDetected = (distance_mm < DANGER_DISTANCE);
      zoneFrontRight.warningDetected = (distance_mm < WARNING_DISTANCE);
      zoneFrontRight.lastUpdate = now;
    }
  }
  // Left (90-180°)
  else if (angle_deg >= ZONE_LEFT_MIN && angle_deg <= ZONE_LEFT_MAX) {
    if (distance_mm < zoneLeft.minDistance) {
      zoneLeft.minDistance = distance_mm;
      zoneLeft.lastUpdate = now;
    }
  }
  // Back (180-270°)
  else if (angle_deg >= ZONE_BACK_MIN && angle_deg <= ZONE_BACK_MAX) {
    if (distance_mm < zoneBack.minDistance) {
      zoneBack.minDistance = distance_mm;
      zoneBack.lastUpdate = now;
    }
  }
  // Right (270-330°)
  else if (angle_deg >= ZONE_RIGHT_MIN && angle_deg <= ZONE_RIGHT_MAX) {
    if (distance_mm < zoneRight.minDistance) {
      zoneRight.minDistance = distance_mm;
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
  // Minimal logging
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("[LIDAR ERROR] ");
  Serial.println(lidar->resultCodeToString(code));
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  // Optional
}

// ========== STATUS REPORT ==========
void printStatus() {
  if (autonomousMode == AUTO_DISABLED) return;
  
  Serial.println("\n========== AUTO STATUS ==========");
  
  Serial.print("Mode: ");
  switch(autonomousMode) {
    case AUTO_EXPLORING: Serial.println("EXPLORING"); break;
    case AUTO_AVOIDING_OBSTACLE: Serial.println("AVOIDING OBSTACLE"); break;
    case AUTO_ROTATING_TO_CLEAR: Serial.println("ROTATING"); break;
    case AUTO_BACKING_UP: Serial.println("BACKING UP"); break;
    default: Serial.println("UNKNOWN"); break;
  }
  
  Serial.println("\nZone Distances:");
  Serial.print("  F-Left:  "); Serial.print(zoneFrontLeft.minDistance, 0); Serial.print("mm");
  if (zoneFrontLeft.dangerDetected) Serial.print(" [DANGER]");
  else if (zoneFrontLeft.warningDetected) Serial.print(" [WARN]");
  Serial.println();
  
  Serial.print("  F-Right: "); Serial.print(zoneFrontRight.minDistance, 0); Serial.print("mm");
  if (zoneFrontRight.dangerDetected) Serial.print(" [DANGER]");
  else if (zoneFrontRight.warningDetected) Serial.print(" [WARN]");
  Serial.println();
  
  Serial.print("  Left:    "); Serial.print(zoneLeft.minDistance, 0); Serial.println("mm");
  Serial.print("  Right:   "); Serial.print(zoneRight.minDistance, 0); Serial.println("mm");
  Serial.print("  Back:    "); Serial.print(zoneBack.minDistance, 0); Serial.println("mm");
  
  Serial.print("\nObstacle Count: ");
  Serial.println(consecutiveObstacles);
  
  Serial.println("=================================\n");
}