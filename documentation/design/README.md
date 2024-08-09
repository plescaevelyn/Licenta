# Design Documentation

This document provides an overview of the design phase of our robotic project, covering the system architecture, software and hardware design, and the different components used in the development.

## 1. System Architecture

The entire system is powered using a 2S LiPo battery regulated by a Step-Down Buck Converter. This was selected to meet the specifications of the Raspberry Pi 5, which requires a 5V and 5A input.

### Components and Data Flow
- **NXP FRDM KE-16Z Development Board**: Receives input from the IMU (Gyroscope and Accelerometer), Lidar, and the radio controller. The processed data from the radio controller is used to control the motor driver, managing the driving and steering of the robot.
- **Raspberry Pi 5**: Receives Lidar and IMU data from the NXP development board and acquires additional data from the wheel encoder. This data is processed using ROS and a SLAM framework.

![System Architecture](path_to_image/system_architecture.png)
*Figure 1: System Architecture*

## 2. Software Design

### Overview
The software design involves two main systems running on separate boards: the NXP FRDM KE-16Z and the Raspberry Pi 5, with data transmitted serially between them.

#### NXP FRDM KE-16Z Development Board
- **Operating System**: FreeRTOS is used for task scheduling.
- **Drivers and Applications**:
  - **UART, I2C, GPIO, ADC, FlexTimer**: Used for communication with sensors, remote controllers, and controlling the ESC and servo motor.
  - **Applications**: Motor control, data acquisition from sensors, and remote control command processing.

#### Raspberry Pi 5
- **Operating System**: Ubuntu 24.04 LTS, chosen for compatibility with ROS2 Jazzy Jalisco.
- **Drivers and Applications**:
  - **GPIO**: Used for acquiring data from the wheel encoder.
  - **Applications**: Data acquisition, SLAM processing with GMapping, and data visualization in Rviz.

![Software Design](path_to_image/software_design.png)
*Figure 2: Software Design*

## 3. Hardware Design

### Robot Structure
The robot's components are organized across three levels:
1. **First Level**: Contains the motor ESC, servomotor, battery, and Power Distribution Board. The wheel encoder is attached to the robot’s axis.
2. **Second Level**: Houses the Power Management system, including the Step-Down Buck Converter and Wago connectors for power distribution. The FlySky FS-i6 radio controller receiver is also placed here.
3. **Third Level**: Hosts the NXP FRDM KE-16Z development board, Raspberry Pi 5, IMU, and Lidar, all connected accordingly.

![Hardware Design](path_to_image/hardware_design.png)
*Figure 3: The Developed Robot*

## 4. Software Modules

### NXP FRDM KE-16Z Modules
- **rbwsFreeRTOS**: Manages real-time operating tasks.
- **rbwsMotorControl**: Handles motor control operations.
- **rbwsPWT**: Manages pulse width timers.
- **rbwsInertialMeasurementUnit**: Processes IMU data.
- **rbwsKalmanFilter**: Applies filtering techniques to sensor data.
- **rbwsLidarA1M8**: Manages Lidar data processing.

### Raspberry Pi 5 Modules
- **imu_data**: Processes IMU data.
- **lidar_data**: Handles Lidar data.
- **encoder_data**: Acquires and processes data from the wheel encoder.
- **common_serial_service**: Manages serial communication between boards.
- **my_gmapping_package**: Implements the GMapping SLAM algorithm for mapping and localization.

## 5. Activity Diagrams

Detailed activity diagrams for the different modules are available in the imgs folder for all the implemented modules.
---

This README provides a comprehensive look at the design process, from system architecture to software and hardware integration, and serves as a guide to understanding the development and implementation of the robotic project.
