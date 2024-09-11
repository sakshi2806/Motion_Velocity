#include <Arduino.h>
#include <MPU9250_WE.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68

const int intPin = 4;
volatile bool motionDetected = false;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

unsigned long startTime = 0;
unsigned long currentTime = 0;
float velocity = 0;
bool isMoving = false;
float prevAccelerationX = 0;

void motionISR();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else {
    Serial.println("MPU9250 is connected");
  }

  Serial.println("Position your MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setIntPinPolarity(MPU9250_ACT_HIGH);
  myMPU9250.enableIntLatch(true);
  myMPU9250.enableClearIntByAnyRead(true);
  myMPU9250.enableInterrupt(MPU9250_WOM_INT);
  myMPU9250.setWakeOnMotionThreshold(50);
  myMPU9250.enableWakeOnMotion(MPU9250_WOM_ENABLE, MPU9250_WOM_COMP_DISABLE);
  attachInterrupt(digitalPinToInterrupt(intPin), motionISR, RISING);
  Serial.println("Turn your MPU9250 and see what happens...");
}

void loop() {
  if(motionDetected) {
    byte source = myMPU9250.readAndClearInterrupts();
    if(myMPU9250.checkInterrupt(source, MPU9250_WOM_INT)){
      Serial.println("Motion detected");
      if (!isMoving) {
        startTime = millis();
        velocity = 0;
        isMoving = true;
      }
    }
    motionDetected = false;
    myMPU9250.readAndClearInterrupts();
  }

  if (isMoving) {
    currentTime = millis();
    float deltaTime = (currentTime - startTime) / 1000.0; // convert to seconds

    xyzFloat accel = myMPU9250.getGValues();
    float accelerationX = accel.x * 9.81; // convert to m/s^2 if necessary

    // Integrate acceleration to get velocity
    velocity += (prevAccelerationX + accelerationX) / 2 * deltaTime; // Trapezoidal integration
    prevAccelerationX = accelerationX; // Store current acceleration for next iteration
    startTime = currentTime;

    // Print current velocity
    Serial.print("Current Velocity: ");
    Serial.print(velocity);
    Serial.println(" m/s");

    // Check if motion has stopped by examining acceleration
    if (abs(accelerationX) < 1.0) { // adjust the threshold as needed
      isMoving = false;
      Serial.print("Motion stopped. Final Velocity: ");
      Serial.print(velocity);
      Serial.println(" m/s");
    }
  }
}

void motionISR() {
  motionDetected = true;
}
