/*

  - Uses AMG8833 8x8 IR array to find hottest object
  - Uses pan/tilt servos on the camera mount to track the heat source
  - Automatically moves toward the heat source whenever it sees one
  - Stops and scans when no valid hot target is visible

  Wiring:
  AMG8833:
    VIN -> 5V
    GND -> GND
    SDA -> A4
    SCL -> A5

  Pan/Tilt Servos:
    Pan  signal -> D9
    Tilt signal -> D10
    Servo power -> 5V and GND (shared with Arduino, or separate 5–6V with common GND)

  L298N Motor Driver:
    Battery +   -> +12V on L298N
    Battery -   -> GND on L298N
    L298N GND   -> Arduino GND (MUST be common)

    Left motor  -> OUT1 / OUT2
    Right motor -> OUT3 / OUT4

    ENA -> D5   (PWM, left speed)
    IN1 -> D4   (left direction)
    IN2 -> D3   (left direction)

    ENB -> D6   (PWM, right speed)
    IN3 -> D7   (right direction)
    IN4 -> D8   (right direction)

  L298N jumpers:
    - Remove ENA jumper
    - Remove ENB jumper
    - LEAVE the 5V/12V (regulator) jumper ON
*/

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// L298N MOTOR DRIVER PINS
// ---------------------------------------------------------------------------

// Left motor (Motor A on L298N)
const int ENA = 5;   // PWM-capable pin - left motor speed
const int IN1 = 4;   // direction pin
const int IN2 = 3;   // direction pin

// Right motor (Motor B on L298N)
const int ENB = 6;   // PWM-capable pin - right motor speed
const int IN3 = 7;   // direction pin
const int IN4 = 8;   // direction pin

// Pan/tilt servos for the camera mount
const int PAN_SERVO_PIN  = 9;   // servo on horizontal axis
const int TILT_SERVO_PIN = 10;  // servo on vertical axis

// ---------------------------------------------------------------------------
// Heat-seeker & servo tuning parameters
// ---------------------------------------------------------------------------

Adafruit_AMG88xx amg;
Servo servoPan;
Servo servoTilt;

// AMG8833 array has 8x8 = 64 pixels
float pixels[64];

// Temperature threshold for "valid" target (deg C)
const float MIN_TARGET_TEMP = 25.0;   // adjust as needed (lighter vs warm hand)

// Pan/tilt servo ranges (degrees)
const int PAN_CENTER   = 90;
const int PAN_MIN      = 30;
const int PAN_MAX      = 150;

const int TILT_CENTER  = 0;   // adjust if your mount is angled
const int TILT_MIN     = 0;
const int TILT_MAX     = 60;

// Proportional gain from image column to pan change
const float PAN_KP = 5.0;   // deg per column error (tune this)

// Simple scanning behavior
const int SCAN_STEP_DEG   = 2;
bool scanRight = true;

// Movement parameters
const int DRIVE_SPEED   = 160;   // 0–255
const int TURN_SPEED    = 150;
const int PAN_DEAD_BAND = 20;    // degrees from center before we start turning

// State flags
bool targetVisible = false;

// Track current servo positions in software
int currentPan  = PAN_CENTER;
int currentTilt = TILT_CENTER;

// ---------------------------------------------------------------------------
// Motor control helper functions (L298N VERSION)
// ---------------------------------------------------------------------------

void tankStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void tankForward(uint8_t speedVal) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void tankBackward(uint8_t speedVal) {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void tankTurnRight(uint8_t speedVal) {
  // Left motor forward, right motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void tankTurnLeft(uint8_t speedVal) {
  // Right motor forward, left motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

// ---------------------------------------------------------------------------
// Servo helper functions
// ---------------------------------------------------------------------------

void setPan(int angle) {
  currentPan = constrain(angle, PAN_MIN, PAN_MAX);
  servoPan.write(currentPan);
}

void setTilt(int angle) {
  currentTilt = constrain(angle, TILT_MIN, TILT_MAX);
  servoTilt.write(currentTilt);
}

// Scan back and forth when no target is visible
void scanForTarget() {
  if (scanRight) {
    setPan(currentPan + SCAN_STEP_DEG);
    if (currentPan >= PAN_MAX) {
      scanRight = false;
    }
  } else {
    setPan(currentPan - SCAN_STEP_DEG);
    if (currentPan <= PAN_MIN) {
      scanRight = true;
    }
  }
}

// ---------------------------------------------------------------------------
// Thermal tracking: read AMG8833, find hottest pixel, aim servos
// ---------------------------------------------------------------------------

void updateThermalTracking() {
  // Read temperatures from all 64 sensors on AMG8833
  amg.readPixels(pixels);

  // Initialize temperature reading
  float maxTemp = -1000.0;
  // -1 means nothing found yet (code has not compared any temperatures yet)
  int maxIndex = -1;

  // For loop that scans through all 64 pixels to find hottest one
  // Once this loop finishes, 'maxTemp' will have the value of the highest temperature seen by the AMG8833
  // and 'maxIndex' will have the position of the object with the highest temperature seen by the AMG8833
  for (int i = 0; i < 64; i++) {
    if (pixels[i] > maxTemp) {
      maxTemp = pixels[i];
      maxIndex = i;
    }
  }

  // Decide if the highest temperature found by AMG8833 is "hot enough"
  if (maxIndex < 0 || maxTemp < MIN_TARGET_TEMP) {
    // No valid hot target
    targetVisible = false;
    return;
  }

  targetVisible = true;

  // Obtaining coordinates (row and column) of hottest object
  // Division provides row number (0-7)
  int row = maxIndex / 8;
  // Modulus provides column number (0-7)
  int col = maxIndex % 8;
  // At this point, you have a vertical coordinate and a horizontal coordinate

  // Calculating how far target is off-center. If this evaluates to ~0, then 
  // target is centered
  float colError = (float)col - 3.5f;
  float rowError = (float)row - 3.5f;

  // Rotate servo based on error calculations above
  // Convert error into degrees of movement by mulitplying by "PAN_KP"
  int panDelta = (int)(colError * PAN_KP);
  setPan(currentPan + panDelta);

  // Same idea as pan, but with tilting up/down
  int tiltDelta = (int)(-rowError * (PAN_KP / 2.0));
  setTilt(currentTilt + tiltDelta);
}

// ---------------------------------------------------------------------------
// Movement logic - ALWAYS CHASE WHEN TARGET VISIBLE
// ---------------------------------------------------------------------------

void updateMovement() {

  // If there's no target visible, break out of this function
  if (!targetVisible) {
    tankStop();
    return;
  }

  // Calculate how off the target is from center
  int panError = currentPan - PAN_CENTER;

  // If target is more off-center than threshold (to the right), turn tank to the right
  if (panError > PAN_DEAD_BAND) {
    tankTurnRight(TURN_SPEED);
    
    // Else if target is more off-center than threshold (to the left), turn tank to the left
  } else if (panError < -PAN_DEAD_BAND) {
    tankTurnLeft(TURN_SPEED);

    // Otherwise, target is within threshold, drive straight to target
  } else {
    tankForward(DRIVE_SPEED);
  }
}

// ---------------------------------------------------------------------------
// Setup and main loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Heat-Seeking Conqueror Tank (AUTO CHASE) starting...");

  // Init motor pins (L298N)
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  tankStop();

  // Init servos
  servoPan.attach(PAN_SERVO_PIN);
  servoTilt.attach(TILT_SERVO_PIN);
  delay(200);
  setPan(PAN_CENTER);
  setTilt(TILT_CENTER);

  // Init AMG8833
  if (!amg.begin()) {
    Serial.println("Could not find a valid AMG8833 sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("AMG8833 initialized.");

  // Give sensor some time to stabilize
  delay(100);
}

void loop() {
  updateThermalTracking(); // Read AMG8833 and point servos at hottest spot

  if (!targetVisible) {
    // No hot target: scan around with the pan servo and stop chassis
    scanForTarget();
    tankStop();
  } else {
    // We see something hot; ALWAYS move toward it
    updateMovement();
  }

  delay(100); // ~10 Hz loop, matches AMG8833 max frame rate
}
