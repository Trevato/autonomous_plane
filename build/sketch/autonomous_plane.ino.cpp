#include <Arduino.h>
#line 1 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino"
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PID_v1_bc.h>

const char *server_url = "http://192.168.159.134:8000";
const char *ssid = "Don't Connect";
const char *password = "3352michigamer";

// Constants
const int leftAileronServoPin = 15;
const int rightAileronServoPin = 14;
const int motorPin = 13; // Change this to the GPIO pin connected to the ESC signal pin
const double rollSetpoint = 0;

// PID tuning parameters
const double Kp = 2.0;
const double Ki = 0.05;
const double Kd = 0.1;

// Objects
MPU6050 mpu(Wire);
Servo leftAileronServo;
Servo rightAileronServo;
Servo motor;
double rollInput, rollOutput, rollError;
PID rollPID(&rollInput, &rollOutput, &rollError, Kp, Ki, Kd, DIRECT);
unsigned long updateMotorTime = 0;
unsigned long updateMotorInterval = 1000; // Update motor speed every 1000 ms

#line 34 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino"
void setup();
#line 64 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino"
void loop();
#line 34 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino"
void setup()
{
  Serial.begin(115200);

  // Initialize MPU-6050
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  // Initialize Servos
  leftAileronServo.attach(leftAileronServoPin);
  rightAileronServo.attach(rightAileronServoPin);
  motor.attach(motorPin);

  // Initialize PID controller
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-90, 90);
  rollPID.SetSampleTime(100);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Attempting to connect to ssid: " + String(ssid));
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop()
{
  // Read MPU-6050 data
  mpu.update();
  rollInput = mpu.getAngleX();

  // Print MPU-6050 outputs
  Serial.print("AccX: ");
  Serial.print(mpu.getAccX());
  Serial.print(", AccY: ");
  Serial.print(mpu.getAccY());
  Serial.print(", AccZ: ");
  Serial.print(mpu.getAccZ());

  Serial.print(", GyroX: ");
  Serial.print(mpu.getGyroX());
  Serial.print(", GyroY: ");
  Serial.print(mpu.getGyroY());
  Serial.print(", GyroZ: ");
  Serial.print(mpu.getGyroZ());

  Serial.print(", AngleX: ");
  Serial.print(mpu.getAngleX());
  Serial.print(", AngleY: ");
  Serial.print(mpu.getAngleY());
  Serial.print(", AngleZ: ");
  Serial.println(mpu.getAngleZ());

  // Compute PID output
  rollPID.Compute();

  // Calculate servo angles based on PID output
  int leftServoAngle = constrain(map(rollOutput, -90, 90, 0, 180), 0, 180);
  int rightServoAngle = constrain(map(-rollOutput, -90, 90, 0, 180), 0, 180);

  // Set servo angles
  leftAileronServo.write(leftServoAngle);
  rightAileronServo.write(rightServoAngle);

  // Get motor speed from the web server
  unsigned long currentTime = millis();
  if (WiFi.status() == WL_CONNECTED && currentTime - updateMotorTime >= updateMotorInterval)
  {
    HTTPClient http;
    http.begin(server_url + String("/get_speed"));
    int httpCode = http.GET();
    if (httpCode > 0)
    {
      String payload = http.getString();

      StaticJsonDocument<64> doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error)
      {
        Serial.println("Failed to parse JSON");
      }
      else
      {
        int speed = doc["speed"];
        int pulse = map(speed, 0, 100, 1000, 2000); // Map speed value to ESC pulse width
        motor.writeMicroseconds(pulse);
        // Serial.println("Motor speed: " + String(speed));
      }
    }
    else
    {
      Serial.println("Error on HTTP request");
    }
    http.end();
    updateMotorTime = currentTime;
  }
}

