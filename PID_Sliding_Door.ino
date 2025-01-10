#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

#define TRIG_PIN 4
#define ECHO_PIN 5
#define SERVO_PIN 9  // Servo motor control pin
Servo slidingDoorServo;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

bool isOpen = 0;

// Create a structure to hold the ranging data
VL53L0X_RangingMeasurementData_t measure;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor with Servo Test");

  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize VL53L0X sensor
  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X! Check wiring.");
    while (1); // Stop execution if sensor initialization fails
  }

  // Attach servo to its pin
  slidingDoorServo.attach(SERVO_PIN);

  // Initialize servo position
  slidingDoorServo.write(142);  // Default closed position
}

int doorPos(){
  // Perform a distance measurement
  lox.rangingTest(&measure, false); // Pass 'true' to print debug info

  // Check if the measurement is valid
  if (measure.RangeStatus != 4) { // RangeStatus 4 means out of range
    return measure.RangeMilliMeter;
  } else return -1;
}

// PID constants
float kp = 0.8;   // Proportional constant
float ki = 0.015;   // Integral constant (can be tuned)
float kd = 0.6;   // Derivative constant

void PIDControl(int setpoint, float* previousError, float* integral) {
  int currentPosition = doorPos();

  while(setpoint < currentPosition - 3 || setpoint > currentPosition + 3){
    if (currentPosition == -1) {
      Serial.println("Error: Position out of range!");
      return;
    }

    // Calculate error
    float error = setpoint - currentPosition;

    // Integral
    *integral += error;

    // Derivative
    float derivative = error - (*previousError);

    // Calculate PID output
    float output = (kp * error + (ki * (*integral)) + (kd * derivative)) + setpoint;

    // Serial.print("output: ");
    // Serial.print(output);

    // Update previous error
    *previousError = error;

    int servoInput = constrain(output * 142 / 274, 23, 160);

    // Move the servo
    slidingDoorServo.write(servoInput);
 
    // Debugging
    // Serial.print(" || Setpoint: ");
    // Serial.print(setpoint);

    currentPosition = doorPos();

    // Serial.print(" mm || Current: ");
    Serial.println(currentPosition);
    // Serial.print(" mm || Servo Input: ");
    // Serial.print(servoInput);
    // Serial.print(" || error sebelumnya: ");
    // Serial.print(*previousError);
    // Serial.print(" mm || integral: ");
    // Serial.println(*integral);

    delay(50);
  }

  for(int i = 0; i < 25; i++)
    Serial.println(doorPos()), delay(100);
}

float previousErrorOpen = 0;
float integralOpen = 0;

void PIDopen() {
  PIDControl(52, &previousErrorOpen, &integralOpen);  // Open position
  delay(250);

  Serial.println("Pintu telah dibuka!");
}

float previousErrorClose = 0;
float integralClose = 0;

void PIDclose() {
  PIDControl(269, &previousErrorClose, &integralClose);  // Closed position
  delay(250);

  Serial.println("Pintu telah ditutup!");
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  int duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.344 / 2; // Convert to mm
  return distance;
}

void loop() {  
  if(getDistance() < 115) {
    if(!isOpen) PIDopen(), isOpen = 1;
  }
  else {
    if(isOpen) PIDclose(), isOpen = 0;
  }

  delay(100);
}