# Autonomous Plane

*Autonomous plane using an ESP32.*

<img src="https://github.com/user-attachments/assets/27ce8b75-fb41-4a81-b79c-c0affe1f82ba" />


## Airleron control:

PID controller takes the ouput from a MPU6050 accelerometer and gyroscope and adjusts the ailerons using servos to control roll angle.

## Speed control:

Currently the plane is only controlled via motor ouput power. We are testing the PID controller to see how it responds before we begin testing full control. The ESP32 does repeated GET requests to a FastAPI backend which returns an integer for the motor output. A user can change this output with a React frontend built on top. This **is not** the best way of doing this but it only took like 5 minutes to write.
