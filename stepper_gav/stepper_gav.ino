#include <AccelStepper.h>

// Pin definitions untuk Motor 1
#define enablePin1 23
#define dirPin1 4
#define stepPin1 2

// Pin definitions untuk Motor 2
#define enablePin2 19
#define dirPin2 18
#define stepPin2 32

// Pin definitions untuk Motor 3
#define enablePin3 27
#define dirPin3 14
#define stepPin3 12

// Pin definitions untuk Motor 4
#define enablePin4 33
#define dirPin4 25
#define stepPin4 26

// Inisialisasi AccelStepper untuk 4 motor (DRIVER mode)
AccelStepper motor4(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor3(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper motor1(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper motor2(AccelStepper::DRIVER, stepPin4, dirPin4);

// Kecepatan dan akselerasi motor (TB6600 dengan 1/8 microstep)
// Dengan 1/8 microstep: 1 putaran = 200 x 8 = 1600 steps
const int maxSpeed = 6400;  // ~4 putaran/detik (6400/1600 = 4 RPS)
const int acceleration = 3200;  // Akselerasi tinggi untuk respons cepat

void setup() {
  Serial.begin(115200);
  
  // Setup enable pins sebagai output dan aktifkan motor (LOW = enable)
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(enablePin3, OUTPUT);
  pinMode(enablePin4, OUTPUT);
  
  digitalWrite(enablePin1, LOW);
  digitalWrite(enablePin2, LOW);
  digitalWrite(enablePin3, LOW);
  digitalWrite(enablePin4, LOW);
  
  // Konfigurasi motor 1
  motor1.setMaxSpeed(maxSpeed);
  motor1.setAcceleration(acceleration);
  
  // Konfigurasi motor 2
  motor2.setMaxSpeed(maxSpeed);
  motor2.setAcceleration(acceleration);
  
  // Konfigurasi motor 3
  motor3.setMaxSpeed(maxSpeed);
  motor3.setAcceleration(acceleration);
  
  // Konfigurasi motor 4
  motor4.setMaxSpeed(maxSpeed);
  motor4.setAcceleration(acceleration);
  
  Serial.println("Omniwheel Controller Ready");
  Serial.println("Commands: w=forward, s=backward, a=left, d=right");
  Serial.println("          q=rotate-left, e=rotate-right, x=stop");
}

void loop() {
  // Cek apakah ada input dari Serial
  if (Serial.available() > 0) {
    char command = Serial.read();
    // Abaikan karakter newline dan carriage return
    if (command != '\n' && command != '\r') {
      handleCommand(command);
    }
  }
  
  // Update semua motor - gunakan runSpeed() untuk continuous rotation
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
}

void handleCommand(char cmd) {
  switch(cmd) {
    case 'w': // Maju
      moveForward();
      Serial.println("Moving Forward");
      break;
      
    case 's': // Mundur
      moveBackward();
      Serial.println("Moving Backward");
      break;
      
    case 'a': // Kiri (strafe)
      strafeLeft();
      Serial.println("Strafing Left");
      break;
      
    case 'd': // Kanan (strafe)
      strafeRight();
      Serial.println("Strafing Right");
      break;
      
    case 'q': // Rotasi kiri
      rotateLeft();
      Serial.println("Rotating Left");
      break;
      
    case 'e': // Rotasi kanan
      rotateRight();
      Serial.println("Rotating Right");
      break;
      
    case 'x': // Stop
      stopAllMotors();
      Serial.println("Stop");
      break;
      
    default:
      Serial.println("Unknown command");
      break;
  }
}

// Fungsi gerakan omniwheel
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
  // motor1.setSpeed(-maxSpeed);
  // motor2.setSpeed(maxSpeed);
  // motor3.setSpeed(-maxSpeed);
  // motor4.setSpeed(maxSpeed);

  motor1.setSpeed(-maxSpeed);
  motor2.setSpeed(-maxSpeed);
  motor3.setSpeed(-maxSpeed);
  motor4.setSpeed(-maxSpeed);
}

void rotateRight() {
  // motor1.setSpeed(maxSpeed);
  // motor2.setSpeed(-maxSpeed);
  // motor3.setSpeed(maxSpeed);
  // motor4.setSpeed(-maxSpeed);
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