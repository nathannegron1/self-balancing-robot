# Self-Balancing Robot

A two-wheel self-balancing robot built with an Arduino Uno, MPU6050 IMU, encoder-equipped DC motors, and cascaded feedback control.

![Self-balancing robot](Media/robot.jpg)

## Overview

The robot estimates its tilt angle using accelerometer and gyroscope data from an MPU6050 combined with a complementary filter. An inner PID controller continuously adjusts motor output to maintain balance.

Encoder feedback is used in an outer velocity-control loop that modifies the desired balance angle. This allows the robot to recover from disturbances and perform controlled forward and backward motion. An IR remote provides forward, reverse, and stop commands.

## Features

- Sustained hands-off self-balancing
- Recovery from forward and backward disturbances
- Encoder-based wheel-speed feedback
- IR-controlled forward, reverse, and stop commands
- Complementary-filter sensor 
- Cascaded balance and velocity control

## Hardware

- Arduino Uno R3
- MPU6050 accelerometer/gyroscope
- TB6612FNG dual motor driver
- Two 6 V DC gearmotors with encoders
- 40 mm wheels
- IR receiver and remote

## Control Architecture

The controller uses two feedback loops:

1. **Inner balance loop:** A PID controller regulates the robot's pitch angle using the filtered MPU6050 measurement.
2. **Outer velocity loop:** The wheel encoders measure how fast the robot is moving. If the measured speed differs from the commanded speed, the outer control loop slightly changes the robot’s target lean angle. The inner PID controller then makes the robot lean and drive in the required direction until the speed error is reduced.

This structure allows the robot to balance independently while also responding to commanded motion.

## Firmware

The complete Arduino firmware is available in [`Firmware/self_balancing_robot.ino`](Firmware/self_balancing_robot.ino).