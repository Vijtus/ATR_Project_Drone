# Quadcopter Flight Controller - Project Documentation

## Requirements
The aim of this project is to create a minimal flight controller for a quadcopter from scratch. Quadcopters, unlike classical planes, do not have any passive aerodynamic stability. Because of this, active control is needed in order to maintain them level and stable. We aim to create the hardware and firmware to perform this stabilization.

### Hardware Implementation Details
We decided to use an F450 frame since a large frame means that the drone will be easier to stabilize.

<img src="https://www.unmannedtechshop.co.uk/wp-content/uploads/1970/01/products-F450_quadcopter_frame__02614.1444739145.1280.1280.jpg" width=500>

The motors are ReadytoSky 920kv BLDCs, and the ESCs are EMAX BLHeli 30A. These are the same ones that were used on the original drone build around this frame, and we just decided to reuse them. For the controller, we decided to use the Raspberry Pi Pico, due to its wide range of peripherals and easy to use yet powerful C SDK. In order for the drone to understand its position in 3D space (more specifically, its angular rates), we chose the LSM6DSOX IMU. This IMU has very small drift and noise, which allows us to use it with minimal filtering in software. The remote is a Flysky i6 transmitter and receiver; here, we used IBUS in order to read out the channel values. The controller was put together on a piece of perf board and attached to the drone.

<img src="https://i.imgur.com/vxv4YNi.jpeg" width=500>

### Software
The stabilization loop is quite simple in theory: just feed the current angular rate from the IMU and the requested one from the remote control, and apply the produced correction signals to the motors. However, implementing the hardware communication to be able to talk to all the inputs at ~200Hz is not trivial. This repository contains the code we used for the controller. We had to create several HAL libraries in order to interact with the different hardware, as well as a Python simulation that helped us better understand the dynamics of the quadrotor.

### Results

<img src="/images/Drone_Footage_1.gif?raw=true" width="800px">
