#include <Wire.h>
#include <Servo.h>

Servo servo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

#define LiftFan 6  // PWM pin for lift fan
#define ThrustFan 5 // PWM pin for thrust fan

#define LeftTrig 13
#define LeftEcho 3
#define RightTrig 11
#define RightEcho 2

#define OFF 0
#define MAX 255


const int MPU6050 = 0x68;//address for MPU communication register
float yawAngle = 0.0;//initialized to 0
float position;//servo position variable 
bool skipUltrasonic = false;
unsigned long skipStartTime = 0;
unsigned long prevTime = 0;

//adjustable vairables and functions
float c = 30; //us bias
float gyroBiasZaxis = 0.1;//gyroscope bias
int k=1; //turning speed constant


void turnLeft(float yawAngle) {
  position = 90-k*(180-yawAngle);
  position = constrain(position, 55, 125);//test edit angle restraints
  servo.write(position);
}

void turnRight(float yawAngle) {
  position = 90+k*(180-yawAngle);
  position = constrain(pos, 55, 125);//test edit angle restraints
  servo.write(pos);
}

// Function to read raw gyroscope Z-axis data
int16_t readGyroZValue() {
    Wire.beginTransmission(MPU6050);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 2, true);

    int16_t gyroZ = (Wire.read() << 8) | Wire.read();
    return gyroZ;
}

// Function to calibrate gyroscope Z-axis bias
float calibrateGyroZ() {
    int numberSamples = 500;
    long sum = 0;
    for (int i = 0; i < numberSamples; i++) {
        sum += readGyroZValue();
        delay(3);
    }
    return (sum / (float)numSamples) / 131.0; // Convert to dps
}

//function to return distance between sensor and wall
float readDistanceFromSensors(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH, 30000);  // Timeout at 30ms = ~5 meters
  float distance = duration * 0.034 / 2;

  if (duration == 0) return -1;  // No echo received
  return distance;
}


void recalibratePositon(float yawAngle){
  position = 4*(90-yawAngle);
  servo.write(pos);
}

void recalibrate2Position(float usSensorRight, float usSensorLeft){
  float calibrate = usSensorRight-usSensorLeft;
  pos = 90-1*(calibrate);
  servo.write(pos);
}



void setup() {
    Serial.begin(115200);//test between 9600 and 115200
    Wire.begin();

  pinMode(LeftTrig, OUTPUT);
  pinMode(LeftEcho, INPUT);
  pinMode(RightTrig, OUTPUT);
  pinMode(RightEcho, INPUT);
  

    // Initialize MPU-6050
    Wire.beginTransmission(MPU6050);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();

    // Gyro calibration: measure bias at startup
    gyroBiasZaxis = calibrateGyroZ();
    servo.attach(9);

    pinMode(LiftFan, OUTPUT);
    pinMode(ThrustFan, OUTPUT);
    analogWrite(LiftFan, 255);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, 255); // Set thrust fan speed (0-255)
}



void loop() {
float leftDistance = -1;
float rightDistance = -1;
   analogWrite(LiftFan, MAX); // Set thrust fan speed (0-255)
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

if (!skipUltrasonic) {
  leftDistance = readDistanceFromSensors(LeftTrig, LeftEcho);
  rightDistance = readDistanceFromSensors(RightTrig, RightEcho);
} else if (millis() - skipStartTime >= 5000) {
  skipUltrasonic = false; // Resume ultrasonic readings after 5 seconds
};

  // Serial.print("Left Distance: ");
  // if (leftDistance == -1)
  //   Serial.print("No Echo");
  // else
  //   Serial.print(leftDistance);
  // Serial.print(" cm | ");

  // Serial.print("Right Distance: ");
  // if (rightDistance == -1)
  //   Serial.println("No Echo");
  // else
  //   Serial.print(rightDistance);
  // Serial.println(" cm");

  

    int16_t gyroZ = readGyroZValue();
    float gyroZ_dps = (gyroZ / 131.0) - gyroBiasZaxis; // Subtract bias

    yawAngle += gyroZ_dps * dt; // Integrate to get yaw

    /*Serial.print("Yaw Angle: ");
    Serial.println(yaw);
    Serial.print("Servo angle: ");
    Serial.println(pos);*/
    delay(100);

   if (rightDistance > leftDistance+c){
    analogWrite(LiftFan, OFF);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, OFF);
    delay(1000);
    analogWrite(LiftFan, MAX);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, 175);
      while(abs(yawAngle) <180){
        unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;
        int16_t gyroZ = readGyroZValue();
        float gyroZ_dps = (gyroZ / 131.0) - gyroBiasZaxis; // Subtract bias

        yaw += gyroZ_dps * dt; // Integrate to get yaw

        Serial.print("Yaw Angle: ");
        Serial.println(yaw);
        turnRight(yaw);
      }
      yaw=0;
    analogWrite(LiftFan, OFF);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, OFF);
    delay(1000);
    analogWrite(LiftFan, MAX);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, MAX);
      skipUltrasonic = true;
      skipStartTime = millis();
   }

    if (leftDistance > rightDistance+c){
      analogWrite(LiftFan, OFF);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, OFF);
      delay(1000);
        analogWrite(LiftFan, MAX);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, 175);
      while(abs(yaw) <180){
        unsigned long currentTime = millis();
       float dt = (currentTime - prevTime) / 1000.0;
        prevTime = currentTime;
        int16_t gyroZ = readGyroZValue();
        float gyroZ_dps = (gyroZ / 131.0) - gyroBiasZaxis; // Subtract bias

        yaw += gyroZ_dps * dt; // Integrate to get yaw

        Serial.print("Yaw Angle: ");
        Serial.println(yaw);
        turnLeft(yaw);
      }
      yaw=0;
        analogWrite(LiftFan, OFF);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, OFF);
    delay(1000);
    analogWrite(LiftFan, MAX);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, MAX);
      skipUltrasonic = true;
      skipStartTime = millis();
    }
  
  position = (2*yawAngle)+85;
    servo.write(position);
  //recalibrate2Position(rightDistance, leftDistance);
    
}


