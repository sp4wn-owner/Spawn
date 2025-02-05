#include <FastAccelStepper.h>
#include <BluetoothSerial.h>
#include <cmath>
#include <float.h> // For FLT_MAX

BluetoothSerial BTSerial;

// Define the step and direction pins for each motor
#define MOTOR1_STEP_PIN 33
#define MOTOR1_DIR_PIN 32
#define MOTOR2_STEP_PIN 18
#define MOTOR2_DIR_PIN 26
#define MOTOR3_STEP_PIN 23
#define MOTOR3_DIR_PIN 14
#define MOTOR4_STEP_PIN 19
#define MOTOR4_DIR_PIN 27
#define MOTOR5_STEP_PIN 22
#define MOTOR5_DIR_PIN 12
#define MOTOR6_STEP_PIN 21
#define MOTOR6_DIR_PIN 13

// Leadscrew parameters
#define LEADSCREW_PITCH 2.0 // Pitch of the leadscrew in mm
#define STEPS_PER_REV 6400  // Steps per revolution for the stepper motor

int speedVar = 48000;
int accVar = 36000;

// Create a FastAccelStepperEngine object
FastAccelStepperEngine engine = FastAccelStepperEngine();

// Create pointers for each stepper motor
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;
FastAccelStepper *stepper5 = NULL;
FastAccelStepper *stepper6 = NULL;

const float link1Length = 10.0;
const float link2Length = 10.0;
const float link3Length = 10.0;

const float pitchScale = 10.0;   // Adjust as needed
const float rollScale = 10.0;    // Adjust as needed
const float yawScale = 10.0;     // Adjust as needed for Z-axis movement
const float heightScale = 400.0; // Adjust as needed, based on the mechanics of your platform
const float rollMovementScale = 10.0; // Adjust as needed for roll movement
const float pitchMovementScale = 10.0; // Adjust as needed for roll movement

float previousJoint1Angle = 0.0;
float previousJoint2Angle = 0.0;
float previousJoint3Angle = 0.0;
float previousJoint4Angle = 0.0;
float previousJoint5Angle = 0.0;
float previousJoint6Angle = 0.0;

float speedMultiplier = 1.0;
float accelMultiplier = 1.0;

// Function to calculate forward kinematics
void forwardKinematics(float joint1Angle, float joint2Angle, float joint3Angle, float &x, float &y, float &z) {
    x = link1Length * cos(joint1Angle) + link2Length * cos(joint1Angle + joint2Angle) + link3Length * cos(joint1Angle + joint2Angle + joint3Angle);
    y = link1Length * sin(joint1Angle) + link2Length * sin(joint1Angle + joint2Angle) + link3Length * sin(joint1Angle + joint2Angle + joint3Angle);
    z = link3Length * sin(joint3Angle);
}

// Error function
float calculateError(float currentX, float currentY, float currentZ, float targetX, float targetY, float targetZ) {
    return sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2) + pow(targetZ - currentZ, 2));
}

// Gradient descent IK solver
bool gradientDescentIK(float targetX, float targetY, float targetZ, float &joint1Angle, float &joint2Angle, float &joint3Angle) {
    const float learningRate = 0.01;
    const int maxIterations = 1000;
    float error = FLT_MAX;

    for (int i = 0; i < maxIterations; i++) {
        float currentX, currentY, currentZ;
        forwardKinematics(joint1Angle, joint2Angle, joint3Angle, currentX, currentY, currentZ);
        error = calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ);

        if (error < 1e-3) {
            return true; // Solution found
        }

        // Compute gradients for all three angles
        float gradient1 = (forwardKinematics(joint1Angle + learningRate, joint2Angle, joint3Angle, currentX, currentY, currentZ), calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ)) - error;
        float gradient2 = (forwardKinematics(joint1Angle, joint2Angle + learningRate, joint3Angle, currentX, currentY, currentZ), calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ)) - error;
        float gradient3 = (forwardKinematics(joint1Angle, joint2Angle, joint3Angle + learningRate, currentX, currentY, currentZ), calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ)) - error;

        // Update joint angles
        joint1Angle -= learningRate * gradient1;
        joint2Angle -= learningRate * gradient2;
        joint3Angle -= learningRate * gradient3;
    }

    return false; // Solution not found within the maximum number of iterations
}

// Function to convert quaternion to Euler angles
void quaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw) {
    float t0 = 2.0 * (w * x + y * z);
    float t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0, t1);

    float t2 = 2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2);

    float t3 = 2.0 * (w * z + x * y);
    float t4 = 1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(t3, t4);
}

void controlJoints(float joint1Angle, float joint2Angle, float joint3Angle = 0, float joint4Angle = 0, float joint5Angle = 0, float joint6Angle = 0) {
    int move1 = -joint1Angle * pitchScale + joint2Angle * rollScale + joint3Angle * yawScale + joint4Angle * pitchMovementScale + joint5Angle * rollMovementScale;
    int move2 = joint1Angle * pitchScale - joint2Angle * rollScale - joint3Angle * yawScale + joint4Angle * pitchMovementScale + joint5Angle * rollMovementScale;
    int move3 = -joint1Angle * pitchScale - joint2Angle * rollScale - joint3Angle * yawScale - joint4Angle * pitchMovementScale + joint5Angle * rollMovementScale;
    int move4 = joint1Angle * pitchScale + joint2Angle * rollScale - joint3Angle * yawScale - joint4Angle * pitchMovementScale - joint5Angle * rollMovementScale;
    int move5 = -joint1Angle * pitchScale + joint2Angle * rollScale - joint3Angle * yawScale + joint4Angle * pitchMovementScale - joint5Angle * rollMovementScale;
    int move6 = joint1Angle * pitchScale - joint2Angle * rollScale + joint3Angle * yawScale  + joint4Angle * pitchMovementScale - joint5Angle * rollMovementScale;

    // Apply the height offset to each stepper
    int heightMovement = joint6Angle * heightScale;
    move1 += heightMovement;
    move2 += heightMovement;
    move3 += heightMovement;
    move4 += heightMovement;
    move5 += heightMovement;
    move6 += heightMovement;

    // Adjust the speed and acceleration based on the multipliers
    int newSpeed = speedVar * speedMultiplier;
    int newAccel = accVar * accelMultiplier;

    // Move the steppers with the adjusted speed and acceleration
    if (stepper1) {
        stepper1->setSpeedInHz(newSpeed);
        stepper1->setAcceleration(newAccel);
        stepper1->moveTo(move1);
    }
    if (stepper2) {
        stepper2->setSpeedInHz(newSpeed);
        stepper2->setAcceleration(newAccel);
        stepper2->moveTo(move2);
    }
    if (stepper3) {
        stepper3->setSpeedInHz(newSpeed);
        stepper3->setAcceleration(newAccel);
        stepper3->moveTo(move3);
    }
    if (stepper4) {
        stepper4->setSpeedInHz(newSpeed);
        stepper4->setAcceleration(newAccel);
        stepper4->moveTo(move4);
    }
    if (stepper5) {
        stepper5->setSpeedInHz(newSpeed);
        stepper5->setAcceleration(newAccel);
        stepper5->moveTo(move5);
    }
    if (stepper6) {
        stepper6->setSpeedInHz(newSpeed);
        stepper6->setAcceleration(newAccel);
        stepper6->moveTo(move6);
    }
}

void moveHeadToPosition(float targetX, float targetY, float targetZ, float roll, float pitch, float yaw) {
    float joint1Angle = previousJoint1Angle; // Initial guess
    float joint2Angle = previousJoint2Angle; // Initial guess
    float joint3Angle = previousJoint3Angle; // Initial guess for third link

    if (gradientDescentIK(targetX, targetY, targetZ, joint1Angle, joint2Angle, joint3Angle)) {
        // Call the function to control the joints with the new angles
        controlJoints(joint1Angle, joint2Angle, joint3Angle, pitch, roll, yaw, targetZ);

        // Save the new angles as the previous angles for the next iteration
        previousJoint1Angle = joint1Angle;
        previousJoint2Angle = joint2Angle;
        previousJoint3Angle = joint3Angle; // Update this as well
    } else {
        Serial.println("Target position is out of reach.");
    }
}

void setup() {
    Serial.begin(115200);
    BTSerial.begin("NECK_BT"); // Start Bluetooth with a name "ESP32_BT"

    // Initialize the stepper engine
    engine.init();

    // Create and configure the stepper motors
    stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);
    if (stepper1)
    {
        stepper1->setDirectionPin(MOTOR1_DIR_PIN);
        stepper1->setEnablePin(25);
        stepper1->setAutoEnable(true);
        stepper1->setSpeedInHz(speedVar);
        stepper1->setAcceleration(accVar);
    }

    stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);
    if (stepper2)
    {
        stepper2->setDirectionPin(MOTOR2_DIR_PIN);
        stepper2->setEnablePin(25);
        stepper2->setAutoEnable(true);
        stepper2->setSpeedInHz(speedVar);
        stepper2->setAcceleration(accVar);
    }

    stepper3 = engine.stepperConnectToPin(MOTOR3_STEP_PIN);
    if (stepper3)
    {
        stepper3->setDirectionPin(MOTOR3_DIR_PIN);
        stepper3->setEnablePin(25);
        stepper3->setAutoEnable(true);
        stepper3->setSpeedInHz(speedVar);
        stepper3->setAcceleration(accVar);
    }

    stepper4 = engine.stepperConnectToPin(MOTOR4_STEP_PIN);
    if (stepper4)
    {
        stepper4->setDirectionPin(MOTOR4_DIR_PIN);
        stepper4->setEnablePin(25);
        stepper4->setAutoEnable(true);
        stepper4->setSpeedInHz(speedVar);
        stepper4->setAcceleration(accVar);
    }

    stepper5 = engine.stepperConnectToPin(MOTOR5_STEP_PIN);
    if (stepper5)
    {
        stepper5->setDirectionPin(MOTOR5_DIR_PIN);
        stepper5->setEnablePin(25);
        stepper5->setAutoEnable(true);
        stepper5->setSpeedInHz(speedVar);
        stepper5->setAcceleration(accVar);
    }

    stepper6 = engine.stepperConnectToPin(MOTOR6_STEP_PIN);
    if (stepper6)
    {
        stepper6->setDirectionPin(MOTOR6_DIR_PIN);
        stepper6->setEnablePin(25);
        stepper6->setAutoEnable(true);
        stepper6->setSpeedInHz(speedVar);
        stepper6->setAcceleration(accVar);
    }

    delay(2000);
    // Reset the position of each stepper motor to 0
    if (stepper1) stepper1->setCurrentPosition(0);
    if (stepper2) stepper2->setCurrentPosition(0);
    if (stepper3) stepper3->setCurrentPosition(0);
    if (stepper4) stepper4->setCurrentPosition(0);
    if (stepper5) stepper5->setCurrentPosition(0);
    if (stepper6) stepper6->setCurrentPosition(0);
}

void loop() {
  if (BTSerial.available()) {
    String jsonString = BTSerial.readStringUntil('\n');
    
    // Parse JSON
    StaticJsonDocument<256> doc; // Adjust size according to your JSON structure
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    
    // Extract values from JSON (assuming JSON has fields like "x", "y", "z", "w", "x", "y", "z")
    float targetX = doc["x"];
    float targetY = doc["y"];
    float targetZ = doc["z"];
    float quatW = doc["quaternion"][0];
    float quatX = doc["quaternion"][1];
    float quatY = doc["quaternion"][2];
    float quatZ = doc["quaternion"][3];

    float roll, pitch, yaw;
    quaternionToEuler(quatW, quatX, quatY, quatZ, roll, pitch, yaw);

    // Use the parsed values
    moveHeadToPosition(targetX, targetY, targetZ, roll, pitch, yaw);
  }
}
