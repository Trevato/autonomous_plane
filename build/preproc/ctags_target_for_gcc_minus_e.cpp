# 1 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino"
/*



*/
# 5 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino"
# 6 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino" 2
# 7 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino" 2
# 8 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino" 2
# 9 "C:\\Users\\trevo\\Desktop\\autonomous_plane\\autonomous_plane.ino" 2

// Constants
const int leftAileronServoPin = 13; // Change this to the GPIO pin connected to the left aileron servo
const int rightAileronServoPin = 14; // Change this to the GPIO pin connected to the right aileron servo
const double rollSetpoint = 0; // Roll angle setpoint (0 degrees)

// PID tuning parameters
const double Kp = 2.0;
const double Ki = 0.05;
const double Kd = 0.1;

// Objects
MPU6050 mpu(Wire);
Servo leftAileronServo;
Servo rightAileronServo;
double rollInput, rollOutput, rollError;
PID rollPID(&rollInput, &rollOutput, &rollError, Kp, Ki, Kd, 0);

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize MPU-6050
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  // Initialize Servos
  leftAileronServo.attach(leftAileronServoPin);
  rightAileronServo.attach(rightAileronServoPin);

  // Initialize PID controller
  rollPID.SetMode(1);
  rollPID.SetOutputLimits(-90, 90); // Limit servo angle correction to -90 to 90 degrees
  rollPID.SetSampleTime(20); // Set PID sample time to 20ms
}

void loop() {
  // Read MPU-6050 data
  mpu.update();
  rollInput = mpu.getAngleX(); // Get roll angle

  // Compute PID output
  rollPID.Compute();

  // Calculate servo angles based on PID output
  int leftServoAngle = ((map(rollOutput, -90, 90, 0, 180))<(0)?(0):((map(rollOutput, -90, 90, 0, 180))>(180)?(180):(map(rollOutput, -90, 90, 0, 180))));
  int rightServoAngle = ((map(-rollOutput, -90, 90, 0, 180))<(0)?(0):((map(-rollOutput, -90, 90, 0, 180))>(180)?(180):(map(-rollOutput, -90, 90, 0, 180)))); // Invert the output for the right aileron

  // Set servo angles
  leftAileronServo.write(leftServoAngle);
  rightAileronServo.write(rightServoAngle);

  // Add delay if needed
  delay(20);
}
