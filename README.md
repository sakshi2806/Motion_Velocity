# MPU9250 Motion and Velocity Detection with ESP32

## Overview
This project demonstrates how to detect motion and calculate velocity using the MPU9250 sensor with an ESP32 microcontroller. The MPU9250 provides accelerometer data, which is used to calculate the velocity based on the detected motion. When motion is detected, the velocity is calculated using trapezoidal integration, and the results are displayed via serial communication.

## Project Structure
* **Microcontroller:** ESP32
* **Sensor:** MPU9250 (9-axis accelerometer, gyroscope, and magnetometer)
* **Libraries Used**
  * `MPU9250_WE.h`: For interfacing with the MPU9250 sensor.
  * `Wire.h`: For I2C communication between the ESP32 and the sensor.

  The system detects motion using the Wake-on-Motion (WoM) feature of the MPU9250, then calculates and outputs the velocity.

## Hardware Components
  1. **ESP32**  microcontroller
  2. **MPU9250** sensor module
  3. **Jumper wires** and breadboard
  4. **USB cable**  (to connect the ESP32 to your computer)

## Circuit Diagram
  ### Pin Connections:

  * **MPU9250 VCC → ESP32 3.3V**
  * **MPU9250 GND → ESP32 GND**
  * **MPU9250 SDA → ESP32 SDA (GPIO 21)**
  * **MPU9250 SCL → ESP32 SCL (GPIO 22)**
  * **INT Pin (Motion Interrupt) → ESP32 GPIO 4**

## Setup Instructions
  1. **Install Arduino IDE:**
  Download and install the Arduino IDE.
  
  2. **Install Required Libraries:**
  This library is used to interface with the MPU9250 sensor. You can install it from the Arduino Library Manager:
  * Go to **Sketch > Include Library > Manage Libraries.**
  * Search for `MPU9250_WE`s and install it.
  * **Wire Library:** This is included with the Arduino IDE by default.


  3. **Code Upload:**
  * Open the provided code in the Arduino IDE.
  * Select the appropriate **ESP32 board** from **Tools > Board.**
  * Set the correct **COM port** for your ESP32 device.
  * Click on Upload to flash the code to the ESP32.

## How It Works
1. **Initialization:** The program initializes the MPU9250 sensor and configures its accelerometer settings. It enables the **Wake-on-Motion (WoM)** interrupt, which triggers the ESP32 whenever the sensor detects movement.

2. **Motion Detection:** The **motionISR** function is called via an interrupt when motion is detected. This sets the motionDetected flag to true.
3. **Velocity Calculation:**
* The accelerometer data is retrieved from the sensor.
* Using trapezoidal integration, the velocity is calculated based on the change in acceleration over time.
* The velocity is printed to the serial monitor.

## Code Explanation
### Key Variables and Functions:
* **motionISR():** Interrupt Service Routine (ISR) triggered by the MPU9250 when motion is detected.
* **setup():** Initializes the MPU9250 sensor, configures accelerometer settings, and attaches the motion interrupt.
* **loop():** Continuously checks for motion, calculates velocity, and prints the results to the serial monitor.

### Motion and Velocity Calculation:

```cpp
float deltaTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds
xyzFloat accel = myMPU9250.getGValues(); // Get acceleration values
float accelerationX = accel.x * 9.81; // Convert G values to m/s^2
velocity += (prevAccelerationX + accelerationX) / 2 * deltaTime; // Trapezoidal integration
```

### Example Output:

```cpp
MPU9250 is connected
Position your MPU9250 flat and don't move it - calibrating...
Done!
Turn your MPU9250 and see what happens...
Motion detected
Current Velocity: 0.05 m/s
Current Velocity: 0.10 m/s
Motion stopped. Final Velocity: 0.10 m/s
```

## Testing the Project
1. **Upload the Code:** Upload the code to your ESP32 using the Arduino IDE.
2. **Open Serial Monitor:** Open the Serial Monitor in the Arduino IDE to view real-time motion and velocity data.
3. **Move the Sensor:** Shake or move the MPU9250 sensor to detect motion and calculate velocity.

## Troubleshooting
* **MPU9250 does not respond:** Ensure that the sensor is properly connected and powered.
* **No motion detection:** Check the interrupt pin connection and ensure that the MPU9250 is correctly calibrated.

## Author
Sakshi Mishra


