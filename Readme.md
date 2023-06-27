# Autonomous Plane

*Autonomous plane using an ESP32.*

## Airleron control:

PID controller takes the ouput from a MPU6050 accelerometer and gyroscope and adjusts the ailerons using servos to control roll angle.

Click [here](https://drive.google.com/file/d/1l3Bz-jgkgPIahoIUOMaz5pyXz4iv__r2/view?usp=sharing) to see a simulation.

## Speed control:

Currently the plane is only controlled via motor ouput power. We are testing the PID controller to see how it responds before we begin testing full control. The ESP32 does repeated GET requests to a FastAPI backend which returns an integer for the motor output. A user can change this output with a React frontend built on top. This **is not** the best way of doing this but it only took like 5 minutes to write.
