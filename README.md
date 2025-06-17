# Arduino Sensor Modules Collection

This repository contains Arduino code for a variety of sensor modules used in the ops course. Each folder includes code for each sensor module for replication. 

## Table of Contents

* [Distance Sensor Module](#distance-sensor-module)
* [GPS & IMU Sensor Module](#gps--imu-sensor-module)
* [Line Sensor Module](#line-sensor-module)
* [Environment Sensor Module](#environment-sensor-module)
* [Camera Module](#camera-module)
* [Lidar Module](#lidar-module)
* [License](#license)

---

## Distance Sensor Module

This module demonstrates how to read distance data using various types of sensors:

* **Ultrasonic Sensor**
  *Component:* [SparkFun Ultrasonic Sensor](https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40/circuit-3b-distance-sensor)
  Uses sound waves to measure the distance to an object.

* **Infrared Sensor**
  *Component:* [GP2Y0A41SK0F](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf)
  Short-range analog IR sensor ideal for precise detection of nearby objects.

* **Micro LiDAR Module**
  *Component:* [TFMini Plus](https://www.sparkfun.com/tfmini-plus-micro-lidar-module.html)
  A compact time-of-flight distance sensor for higher accuracy and longer range.

---

## GPS & IMU Sensor Module

This module combines GPS positioning with motion and orientation sensing.

* **GPS Module**
  *Component:* [SparkFun GPS Module](https://www.sparkfun.com/sparkfun-gps-breakout-xa1110-qwiic.html)
  Provides latitude, longitude, altitude, and timestamp data via serial connection.

* **IMU (Inertial Measurement Unit)**
  *Component:* [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout](https://www.adafruit.com/product/4646)
  Combines accelerometer, gyroscope, and magnetometer with onboard sensor fusion to provide orientation data (yaw, pitch, roll).

---

## Line Sensor Module

Used for line-following robots and navigation over predefined paths.

* **Line Follower Array**
  *Component:* [SparkFun Line Follower Array](https://www.sparkfun.com/sparkfun-line-follower-array.html)
  8-channel reflectance sensor array to detect line position with high precision.

* **Industrial Scale Line Follower**
  *Component:* [Industrial Grade Line Sensor]()
  (Insert your specific model here.) Designed for robust line tracking in factory environments.

---

## Environment Sensor Module

Collects environmental data and detects physical interactions.

* **Gas, Temperature, Humidity, Pressure Sensor**
  *Component:* [Adafruit BME688]()
  A powerful sensor for collecting air quality, gas levels, temperature, humidity, and pressure data.

* **Collision Detection**
  *Component:* [Roller Microswitch]()
  Simple mechanical switch for detecting contact/collisions with objects.

---

## Camera Module

*Coming Soon*
Includes Arduino-based interface for image or video capture. (Details and code will be added.)

---

## Lidar Module

*Coming Soon*
Standalone Lidar module with real-time 2D/3D scanning support. (Details and code will be added.)

---


