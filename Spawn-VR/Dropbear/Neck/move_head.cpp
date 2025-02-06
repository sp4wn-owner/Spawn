#include <FastAccelStepper.h>
#include <BluetoothSerial.h>
#include <cmath>
#include <float.h> // For FLT_MAX
#include <ArduinoJson.h>

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

int speedVar = 48000;
int accVar = 36000;

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;
FastAccelStepper *stepper5 = NULL;
FastAccelStepper *stepper6 = NULL;

const float link1Length = 10.0;
const float link2Length = 10.0;
const float link3Length = 10.0;

// Assuming these scales relate to how the motors should react to quaternion components
const float qxScale = 10.0;    // Adjust as needed for X-axis rotation
const float qyScale = 10.0;    // Adjust as needed for Y-axis rotation
const float qzScale = 10.0;    // Adjust as needed for Z-axis rotation
const float qwScale = 10.0;    // Adjust as needed for the scalar part of the quaternion
const float heightScale = 400.0; // Adjust as needed, based on the mechanics of your platform

float previousJoint1Angle = 0.0;
float previousJoint2Angle = 0.0;
float previousJoint3Angle = 0.0;

float speedMultiplier = 1.0;
float accelMultiplier = 1.0;

struct TrainingData {
    float targetX, targetY, targetZ;
    float quatW, quatX, quatY, quatZ;
    float joint1Angle, joint2Angle, joint3Angle;
    float currentX, currentY, currentZ;
};

// Simple model parameters for linear regression
float weights[3][7]; // 3 joint angles, each predicted by 7 inputs (x, y, z, w, x, y, z of quaternion)
float learningRate = 0.001;

// Initialize weights to small random values or zeros
void initWeights() {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 7; ++j) {
            weights[i][j] = 0.0; // or random value if you want more variability
        }
    }
}

// Predict joint angles from the current model
void predictAngles(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ, 
                   float &joint1Angle, float &joint2Angle, float &joint3Angle) {
    float inputs[7] = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ};
    
    for (int i = 0; i < 3; ++i) {
        float prediction = 0.0;
        for (int j = 0; j < 7; j++) {
            prediction += weights[i][j] * inputs[j];
        }
        if (i == 0) joint1Angle = prediction;
        else if (i == 1) joint2Angle = prediction;
        else joint3Angle = prediction;
    }
}

// Update model with new data (session learning)
void updateModel(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ, 
                 float joint1Angle, float joint2Angle, float joint3Angle) {
    float inputs[7] = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ};
    float outputs[3] = {joint1Angle, joint2Angle, joint3Angle};
    
    for (int i = 0; i < 3; ++i) { // for each joint angle
        float prediction = 0;
        for (int j = 0; j < 7; j++) {
            prediction += weights[i][j] * inputs[j];
        }
        float error = outputs[i] - prediction;
        
        for (int j = 0; j < 7; j++) {
            weights[i][j] += learningRate * error * inputs[j]; // Gradient descent update rule
        }
    }
}

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

// Compute analytical gradients for efficiency
void computeGradients(float joint1, float joint2, float joint3, float targetX, float targetY, float targetZ, float &grad1, float &grad2, float &grad3) {
    float currentX, currentY, currentZ;
    forwardKinematics(joint1, joint2, joint3, currentX, currentY, currentZ);

    float epsilon = 0.001;
    float x1, y1, z1, x2, y2, z2, x3, y3, z3;

    forwardKinematics(joint1 + epsilon, joint2, joint3, x1, y1, z1);
    grad1 = (calculateError(x1, y1, z1, targetX, targetY, targetZ) - calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ)) / epsilon;

    forwardKinematics(joint1, joint2 + epsilon, joint3, x2, y2, z2);
    grad2 = (calculateError(x2, y2, z2, targetX, targetY, targetZ) - calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ)) / epsilon;

    forwardKinematics(joint1, joint2, joint3 + epsilon, x3, y3, z3);
    grad3 = (calculateError(x3, y3, z3, targetX, targetY, targetZ) - calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ)) / epsilon;
}

// Gradient descent IK solver with improved convergence
bool gradientDescentIK(float targetX, float targetY, float targetZ, float &joint1Angle, float &joint2Angle, float &joint3Angle, std::vector &data) {
    const float learningRate = 0.01;
    const int maxIterations = 1000;
    float error = FLT_MAX;

    for (int i = 0; i < maxIterations; i++) {
        float currentX, currentY, currentZ;
        forwardKinematics(joint1Angle, joint2Angle, joint3Angle, currentX, currentY, currentZ);
        error = calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ);

        if (error < 1e-3) {
            TrainingData newData = {targetX, targetY, targetZ, 0, 0, 0, 0, joint1Angle, joint2Angle, joint3Angle, currentX, currentY, currentZ};
            data.push_back(newData);
            return true;
        }

        float grad1, grad2, grad3;
        computeGradients(joint1Angle, joint2Angle, joint3Angle, targetX, targetY, targetZ, grad1, grad2, grad3);

        // Update joint angles with gradients
        joint1Angle -= learningRate * grad1;
        joint2Angle -= learningRate * grad2;
        joint3Angle -= learningRate * grad3;

        // Simple learning rate adjustment (you might want something more sophisticated)
        if (i > 0 && error > previousError) learningRate *= 0.9; // Decrease learning rate if error increases
        previousError = error;
    }

    return false;
}

// Control function now uses quaternion directly for movement
void controlJoints(float joint1Angle, float joint2Angle, float joint3Angle, float quatW, float quatX, float quatY, float quatZ, float targetZ) {
    int move1 = -joint1Angle * qxScale + joint2Angle * qyScale + joint3Angle * qzScale - quatW * qwScale;
    int move2 = joint1Angle * qxScale - joint2Angle * qyScale - joint3Angle * qzScale - quatW * qwScale;
    int move3 = -joint1Angle * qxScale - joint2Angle * qyScale - joint3Angle * qzScale - quatW * qwScale;
    int move4 = joint1Angle * qxScale + joint2Angle * qyScale - joint3Angle * qzScale - quatW * qwScale;
    int move5 = -joint1Angle * qxScale + joint2Angle * qyScale - joint3Angle * qzScale - quatW * qwScale;
    int move6 = joint1Angle * qxScale - joint2Angle * qyScale + joint3Angle * qzScale - quatW * qwScale;

    // Adjust for height (this assumes Z movement is uniform for all motors)
    int heightMovement = targetZ * heightScale;
    move1 += heightMovement;
    move2 += heightMovement;
    move3 += heightMovement;
    move4 += heightMovement;
    move5 += heightMovement;
    move6 += heightMovement;

    int newSpeed = speedVar * speedMultiplier;
    int newAccel = accVar * accelMultiplier;

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

// Modify moveHeadToPosition to use and update the model
void moveHeadToPosition(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ, std::vector<TrainingData> &data) {
    float predictedJoint1Angle, predictedJoint2Angle, predictedJoint3Angle;
    
    // Predict initial guess with the model
    predictAngles(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, 
                  predictedJoint1Angle, predictedJoint2Angle, predictedJoint3Angle);

    // Use gradient descent as usual but with a better starting point
    if (gradientDescentIK(targetX, targetY, targetZ, predictedJoint1Angle, predictedJoint2Angle, predictedJoint3Angle, data)) {
        controlJoints(predictedJoint1Angle, predictedJoint2Angle, predictedJoint3Angle, quatW, quatX, quatY, quatZ, targetZ);

        // Update model with the solution found
        updateModel(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, 
                    predictedJoint1Angle, predictedJoint2Angle, predictedJoint3Angle);

        previousJoint1Angle = predictedJoint1Angle;
        previousJoint2Angle = predictedJoint2Angle;
        previousJoint3Angle = predictedJoint3Angle;
    } else {
        Serial.println("Target position is out of reach.");
    }
}

void setup() {
    Serial.begin(115200);
    BTSerial.begin("NECK_BT"); // Start Bluetooth with a name "ESP32_BT"

    // Initialize the stepper engine
    engine.init();
    initWeights(); // Initialize model weights

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

std::vector trainingData;
float previousError = FLT_MAX; // Used for learning rate adjustment

void loop() {
    if (BTSerial.available()) {
        String jsonString = BTSerial.readStringUntil('\n');
        
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, jsonString);
        
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
            return;
        }
        
        float targetX = doc["x"];
        float targetY = doc["y"];
        float targetZ = doc["z"];
        float quatW = doc["quaternion"][0];
        float quatX = doc["quaternion"][1];
        float quatY = doc["quaternion"][2];
        float quatZ = doc["quaternion"][3];

        moveHeadToPosition(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, trainingData);
    }
}