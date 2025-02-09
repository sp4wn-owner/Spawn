#include <FastAccelStepper.h>
#include <BluetoothSerial.h>
#include <cmath>
#include <float.h>
#include <ArduinoJson.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <esp_system.h>
#include <SD_MMC.h>
#include <FS.h>
#include <esp_vfs_fat.h>

bool areMotorsConnected = false; // Set to true if motors are connected
bool updateMotors = false;        // Set to true to update motors with solutions
bool autoTrain = true;            // true enables auto training which reprocesses existing solutions to minimize error || false disables auto training. Set to false when live
bool trainWithRandomQuats = true; // Set to false to use real VR tracking data

const float link1Length = 10.0;
const float link2Length = 10.0;
const float link3Length = 10.0;

const float qxScale = 10.0; // Adjust as needed for X-axis rotation
const float qyScale = 10.0; // Adjust as needed for Y-axis rotation
const float qzScale = 10.0; // Adjust as needed for Z-axis rotation
const float qwScale = 10.0; // Adjust as needed for the scalar part of the quaternion

const float standardScale = 10;
const float pitchScale = 10;
const float rollScale = 10;
const float yawScale = 10;
const float rollMovementScale = 10.0;
const float pitchMovementScale = 10.0;
const float heightScale = 400.0;

int speedVar = 48000;
int accVar = 36000;
float speedMultiplier = 1.0;
float accelMultiplier = 1.0;
int maxSteps = 10000;
int minSteps = -10000;

float previousJoint1Angle = 0.0;
float previousJoint2Angle = 0.0;
float previousJoint3Angle = 0.0;
float previousJoint4Angle = 0.0;
float previousJoint5Angle = 0.0;
float previousJoint6Angle = 0.0;

struct TrainingData
{
    float targetX, targetY, targetZ;
    float currentX, currentY, currentZ;
    float quatW, quatX, quatY, quatZ;
    float joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle;
    float error;
};

std::vector<TrainingData> trainingData;

struct IKResult
{
    bool solutionFound;
    float error;
};

BluetoothSerial BTSerial;

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

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;
FastAccelStepper *stepper5 = NULL;
FastAccelStepper *stepper6 = NULL;

// Function prototypes can be here to avoid "implicit declaration" warnings
void initWeights();
void predictAngles(float, float, float, float, float, float, float, float &, float &, float &, float &, float &, float &);
IKResult gradientDescentIK(float, float, float, float &, float &, float &, float &, float &, float &, std::vector<TrainingData> &, float &, float, float, float, float);
unsigned int updateModel(float, float, float, float, float, float, float, float, float, float, float, float, float);
void quaternionToRotationMatrix(float, float, float, float, float[3][3]);
void forwardKinematics(float, float, float, float, float, float, float, float &, float &, float &);
float calculateError(float, float, float, float, float, float, float, float, float, float, float, float, float, float);
void computeGradients(float, float, float, float, float, float, float, float, float, float, float &, float &, float &);
void generateRotationQuaternion(float &, float &, float &, float &);

// Simple model parameters for linear regression
float weights[6][7]; 
float learningRate = 0.1; // Lower rate for more precision, but may take longer

void initWeights()
{
    static bool seeded = false;
    if (!seeded)
    {
        srand(time(NULL));
        seeded = true;
    }

    float range = sqrt(2.0 / (6 + 7));
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 7; ++j)
        {
            weights[i][j] = ((float)rand() / RAND_MAX) * range - (range / 2.0);
        }
    }
}

void checkFreeHeap()
{
    size_t freeHeap = ESP.getFreeHeap();
    Serial.print("Free Heap: ");
    Serial.print(freeHeap);
    Serial.println(" bytes");
}

// Predict joint angles from the current model
void predictAngles(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ,
                   float &joint1Angle, float &joint2Angle, float &joint3Angle, float &joint4Angle, float &joint5Angle, float &joint6Angle)
{
    float inputs[7] = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ};

    // Scales for each joint, reflecting their mechanical influence or sensitivity
    float scales[6] = {pitchScale, rollScale, yawScale, standardScale, standardScale, standardScale};

    for (int i = 0; i < 6; ++i)
    {
        float predictionInRadians = 0.0;
        for (int j = 0; j < 7; j++)
        {
            predictionInRadians += weights[i][j] * inputs[j];
        }

        // Convert prediction from radians to degrees and apply scale
        float predictionInDegrees = predictionInRadians * (180.0 / M_PI) * scales[i];

        // Apply constraints to prevent drift or ensure physical limits
        float minAngle = -180.0; // Example minimum angle, adjust based on your robot's capabilities
        float maxAngle = 180.0;  // Example maximum angle, adjust based on your robot's capabilities

        switch (i)
        {
        case 0:
            joint1Angle = constrain(predictionInDegrees, minAngle, maxAngle);
            break;
        case 1:
            joint2Angle = constrain(predictionInDegrees, minAngle, maxAngle);
            break;
        case 2:
            joint3Angle = constrain(predictionInDegrees, minAngle, maxAngle);
            break;
        case 3:
            joint4Angle = constrain(predictionInDegrees, minAngle, maxAngle);
            break;
        case 4:
            joint5Angle = constrain(predictionInDegrees, minAngle, maxAngle);
            break;
        case 5:
            joint6Angle = constrain(predictionInDegrees, minAngle, maxAngle);
            break;
        }
    }
}

unsigned int updateModel(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ,
                         float joint1Angle, float joint2Angle, float joint3Angle, float joint4Angle, float joint5Angle, float joint6Angle)
{
    static unsigned int updateCount = 0;
    updateCount++;

    Serial.println("Updating Model:");
    Serial.print("Model Update Count: ");
    Serial.println(updateCount); 
    Serial.print("Target Position: ");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.print(targetY);
    Serial.print(", ");
    Serial.println(targetZ);
    Serial.print("Quaternion: ");
    Serial.print(quatW);
    Serial.print(", ");
    Serial.print(quatX);
    Serial.print(", ");
    Serial.print(quatY);
    Serial.print(", ");
    Serial.println(quatZ);
    Serial.print("Joint Angles: ");
    Serial.print(joint1Angle);
    Serial.print(", ");
    Serial.print(joint2Angle);
    Serial.print(", ");
    Serial.print(joint3Angle);
    Serial.print(", ");
    Serial.print(joint4Angle);
    Serial.print(", ");
    Serial.print(joint5Angle);
    Serial.print(", ");
    Serial.println(joint6Angle);

    float inputs[7] = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ};
    float outputs[6] = {
        joint1Angle / ((180.0 / M_PI)),
        joint2Angle / ((180.0 / M_PI)),
        joint3Angle / ((180.0 / M_PI)),
        joint4Angle / ((180.0 / M_PI)),
        joint5Angle / ((180.0 / M_PI)),
        joint6Angle / ((180.0 / M_PI))};

    for (int i = 0; i < 6; ++i)
    { // for each joint angle
        float prediction = 0;
        for (int j = 0; j < 7; j++)
        {
            prediction += weights[i][j] * inputs[j];
        }
        float error = outputs[i] - prediction;

        Serial.print("Joint ");
        Serial.print(i + 1);
        Serial.print(" - Prediction: ");
        Serial.print(prediction);
        Serial.print(", Actual: ");
        Serial.print(outputs[i]);
        Serial.print(", Error: ");
        Serial.println(error);

        for (int j = 0; j < 7; j++)
        {
            weights[i][j] += learningRate * error * inputs[j]; // Gradient descent update rule
        }
    }
    return updateCount;
}

void quaternionToRotationMatrix(float quatW, float quatX, float quatY, float quatZ, float rotMat[3][3])
{
    // Normalize the quaternion
    float norm = sqrt(quatW * quatW + quatX * quatX + quatY * quatY + quatZ * quatZ);
    quatW /= norm;
    quatX /= norm;
    quatY /= norm;
    quatZ /= norm;

    // Compute elements of the rotation matrix
    rotMat[0][0] = 1 - 2 * (quatY * quatY + quatZ * quatZ);
    rotMat[0][1] = 2 * (quatX * quatY - quatZ * quatW);
    rotMat[0][2] = 2 * (quatX * quatZ + quatY * quatW);

    rotMat[1][0] = 2 * (quatX * quatY + quatZ * quatW);
    rotMat[1][1] = 1 - 2 * (quatX * quatX + quatZ * quatZ);
    rotMat[1][2] = 2 * (quatY * quatZ - quatX * quatW);

    rotMat[2][0] = 2 * (quatX * quatZ - quatY * quatW);
    rotMat[2][1] = 2 * (quatY * quatZ + quatX * quatW);
    rotMat[2][2] = 1 - 2 * (quatX * quatX + quatY * quatY);
}

void forwardKinematics(float joint1Angle, float joint2Angle, float joint3Angle, float joint4Angle, float joint5Angle, float joint6Angle,
                       float quatW, float quatX, float quatY, float quatZ, float &x, float &y, float &z)
{
    // Effective length for movement calculations
    const float linkLength = link1Length;

    // Position in local coordinates
    float localX = 0, localY = 0, localZ = 0;
    float angles[6] = {joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle};

    // Adjusting based on the move equations without scales:
    for (int i = 0; i < 6; ++i)
    {
        float angle = angles[i];
        switch (i)
        {
        case 0: // move1: -angleX + angleY + angleZ
            localX -= angle;
            localY += angle;
            localZ += angle;
            break;
        case 1: // move2: angleX - angleY - angleZ
            localX += angle;
            localY -= angle;
            localZ -= angle;
            break;
        case 2: // move3: -angleX - angleY - angleZ
            localX -= angle;
            localY -= angle;
            localZ -= angle;
            break;
        case 3: // move4: angleX + angleY - angleZ
            localX += angle;
            localY += angle;
            localZ -= angle;
            break;
        case 4: // move5: -angleX + angleY - angleZ
            localX -= angle;
            localY += angle;
            localZ -= angle;
            break;
        case 5: // move6: angleX - angleY + angleZ
            localX += angle;
            localY -= angle;
            localZ += angle;
            break;
        }
    }

    float rotMat[3][3];
    quaternionToRotationMatrix(quatW, quatX, quatY, quatZ, rotMat);

    // Apply rotation to get global coordinates
    x = rotMat[0][0] * localX + rotMat[0][1] * localY + rotMat[0][2] * localZ;
    y = rotMat[1][0] * localX + rotMat[1][1] * localY + rotMat[1][2] * localZ;
    z = rotMat[2][0] * localX + rotMat[2][1] * localY + rotMat[2][2] * localZ;
}

float calculateError(float currentX, float currentY, float currentZ,
                     float targetX, float targetY, float targetZ,
                     float quatW, float quatX, float quatY, float quatZ,
                     float targetQuatW, float targetQuatX, float targetQuatY, float targetQuatZ)
{
    // Position Error
    float posError = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2) + pow(targetZ - currentZ, 2));

    // Normalize current quaternion
    float currentNorm = sqrt(quatW * quatW + quatX * quatX + quatY * quatY + quatZ * quatZ);
    quatW /= currentNorm;
    quatX /= currentNorm;
    quatY /= currentNorm;
    quatZ /= currentNorm;

    // Normalize target quaternion
    float targetNorm = sqrt(targetQuatW * targetQuatW + targetQuatX * targetQuatX + targetQuatY * targetQuatY + targetQuatZ * targetQuatZ);
    targetQuatW /= targetNorm;
    targetQuatX /= targetNorm;
    targetQuatY /= targetNorm;
    targetQuatZ /= targetNorm;

    // Orientation Error
    float dotProduct = quatW * targetQuatW + quatX * targetQuatX + quatY * targetQuatY + quatZ * targetQuatZ;
    dotProduct = fmin(1.0f, fmax(dotProduct, -1.0f)); // Clamp to avoid acos domain errors
    float oriError = 2.0f * acosf(fabs(dotProduct));  // Angle in radians

    // Combine errors. Note: Weighting might be necessary based on application specifics
    return posError + oriError;
}

// Compute analytical gradients for efficiency
void computeGradients(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6,
                      float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ,
                      float &grad1, float &grad2, float &grad3, float &grad4, float &grad5, float &grad6)
{
    // A smaller epsilon might be needed for fine control, but this depends on your system's scale
    float epsilon = 0.001; // Adjust based on testing
    float currentX, currentY, currentZ;
    float x1, y1, z1, x2, y2, z2; 

    // Compute current position
    forwardKinematics(joint1, joint2, joint3, joint4, joint5, joint6, quatW, quatX, quatY, quatZ, currentX, currentY, currentZ);
    float baseError = calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, quatW, quatX, quatY, quatZ);

    // Gradient for each joint using central difference for increased accuracy
    for (int i = 0; i < 6; i++)
    {
        float angles[6] = {joint1, joint2, joint3, joint4, joint5, joint6};

        // Perturb up
        angles[i] += epsilon;
        forwardKinematics(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
                          quatW, quatX, quatY, quatZ, x1, y1, z1);
        float errorUp = calculateError(x1, y1, z1, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, quatW, quatX, quatY, quatZ);

        // Perturb down
        angles[i] -= 2 * epsilon; // Reset and perturb down
        forwardKinematics(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
                          quatW, quatX, quatY, quatZ, x2, y2, z2);
        float errorDown = calculateError(x2, y2, z2, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, quatW, quatX, quatY, quatZ);

        // Reset angle
        angles[i] += epsilon; // Reset back to original angle

        float *gradPtr = &grad1 + i;
        *gradPtr = (errorUp - errorDown) / (2 * epsilon); // Central difference
    }
}

// Gradient descent IK solver with improved convergence
IKResult gradientDescentIK(float targetX, float targetY, float targetZ,
                           float &joint1Angle, float &joint2Angle, float &joint3Angle,
                           float &joint4Angle, float &joint5Angle, float &joint6Angle,
                           std::vector<TrainingData> &data, float &learningRate,
                           float quatW, float quatX, float quatY, float quatZ)
{
    const int maxIterations = 1000;
    float error = FLT_MAX;
    float previousError = FLT_MAX;
    bool solutionFound = false;
    float currentX, currentY, currentZ;

    Serial.println("Starting Gradient Descent IK for 6DoF:");
    Serial.print("Target Position: ");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.print(targetY);
    Serial.print(", ");
    Serial.println(targetZ);

    float jointAngles[6] = {joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle};

    for (int i = 0; i < maxIterations; i++)
    {
        // Normalize input quaternion at the start of each iteration to avoid drift
        normalizeQuaternion(quatW, quatX, quatY, quatZ);

        forwardKinematics(jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5],
                          quatW, quatX, quatY, quatZ, currentX, currentY, currentZ);

        // Ensure the quaternion part of the result remains normalized
        normalizeQuaternion(quatW, quatX, quatY, quatZ);

        error = calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, quatW, quatX, quatY, quatZ);

        if (error < .01)
        {
            Serial.println("Solution found within error threshold.");
            Serial.print("Error Rate: ");
            Serial.println(error);
            TrainingData newData = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ,
                                    jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5],
                                    currentX, currentY, currentZ};
            data.push_back(newData);
            solutionFound = true;
            break;
        }

        float gradients[6];
        computeGradients(jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5],
                         targetX, targetY, targetZ, quatW, quatX, quatY, quatZ,
                         gradients[0], gradients[1], gradients[2], gradients[3], gradients[4], gradients[5]);

        for (int j = 0; j < 6; j++)
        {
            jointAngles[j] -= learningRate * gradients[j];
        }

        // Learning rate adjustment
        if (i > 5 && error > previousError)
        {
            learningRate *= 0.9; // Decrease learning rate if error increases
        }
        else if (error < previousError)
        {
            learningRate = min(learningRate * 1.05, 1.0); // Increase learning rate but cap it
        }
        previousError = error;
    }

    if (!solutionFound)
    {

        forwardKinematics(jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5],
                          quatW, quatX, quatY, quatZ, currentX, currentY, currentZ);
        Serial.println("No solution found after max iterations.");
        Serial.print("Final Error: ");
        Serial.println(error);
        Serial.print("Final Position: ");
        Serial.print(currentX);
        Serial.print(", ");
        Serial.print(currentY);
        Serial.print(", ");
        Serial.println(currentZ);
        Serial.print("Final Joint Angles: ");
        for (int i = 0; i < 6; i++)
        {
            Serial.print(jointAngles[i]);
            if (i < 5)
                Serial.print(", ");
        }
        Serial.println();
    }
    else
    {
        Serial.println("Solution found:");
        Serial.print("Final Position: ");
        Serial.print(currentX, 6);
        Serial.print(", ");
        Serial.print(currentY, 6);
        Serial.print(", ");
        Serial.println(currentZ, 6);
        Serial.print("Final Joint Angles: ");
        for (int i = 0; i < 6; i++)
        {
            Serial.print(jointAngles[i], 6);
            if (i < 5)
                Serial.print(", ");
        }
        Serial.println();
        // Convert to motor steps for all 6 motors
        const int stepsPerRevolution = 6400; // Based on your DISTANCE_PER_STEP implying 6400 steps per full revolution at 1/8 microstepping
        const float gearRatio = 1.0;         // Example gear ratio, adjust if applicable
        const int microstepping = 8;

        int steps[6];
        float scales[6] = {pitchScale, rollScale, yawScale, standardScale, standardScale, standardScale}; // Declare scales here

        for (int i = 0; i < 6; i++)
        {
            // Convert radians to steps considering the scale factors
            float totalAngle = jointAngles[i] * scales[i];

            // Convert angle to steps
            steps[i] = int(round((totalAngle / (2 * PI)) * stepsPerRevolution * gearRatio * microstepping));

            // Constrain steps within motor limits
            steps[i] = constrain(steps[i], minSteps, maxSteps);

            Serial.print("Steps for Motor ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println(steps[i]);
        }
    }

    // Update joint angles back to the passed references
    joint1Angle = jointAngles[0];
    joint2Angle = jointAngles[1];
    joint3Angle = jointAngles[2];
    joint4Angle = jointAngles[3];
    joint5Angle = jointAngles[4];
    joint6Angle = jointAngles[5];

    IKResult result;
    result.solutionFound = solutionFound;
    result.error = error;

    return result;
}

// Helper function for quaternion normalization
void normalizeQuaternion(float &w, float &x, float &y, float &z)
{
    float norm = sqrt(w * w + x * x + y * y + z * z);
    if (norm != 0)
    {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
}

// Modify moveHeadToPosition to use and update the model
void moveHeadToPosition(float joint1Angle, float joint2Angle, float joint3Angle,
                        float joint4Angle, float joint5Angle, float joint6Angle,
                        float targetX, float targetY, float targetZ,
                        float quatW, float quatX, float quatY, float quatZ)
{

    // Array to store joint angles
    float jointAngles[6] = {joint1Angle, joint2Angle, joint3Angle,
                            joint4Angle, joint5Angle, joint6Angle};

    // Conversion constants
    const float DISTANCE_PER_STEP = ((2.0 / 6400) / 8);
    const float scales[6] = {standardScale, standardScale, standardScale, standardScale, standardScale, standardScale};

    // Motor limits
    const int maxSteps = 10000;  // Example, adjust based on your hardware
    const int minSteps = -10000; // Example, adjust based on your hardware

    // Calculate steps for each motor
    int steps[6];
    for (int i = 0; i < 6; i++)
    {
        steps[i] = (jointAngles[i] / scales[i]) / DISTANCE_PER_STEP;
        steps[i] = constrain(steps[i], minSteps, maxSteps);
        Serial.print("Updating motors ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(steps[i]);
    }

    // Adjust for height (assuming Z movement is uniform for all motors)
    int heightMovement = targetZ * heightScale;
    for (int i = 0; i < 6; i++)
    {
        steps[i] += heightMovement; // Add height adjustment to each motor step
    }

    // Speed and acceleration settings
    int newSpeed = speedVar * speedMultiplier;
    int newAccel = accVar * accelMultiplier;

    if (stepper1)
    {
        stepper1->setSpeedInHz(newSpeed);
        stepper1->setAcceleration(newAccel);
        stepper1->moveTo(steps[0]);
    }
    if (stepper2)
    {
        stepper2->setSpeedInHz(newSpeed);
        stepper2->setAcceleration(newAccel);
        stepper2->moveTo(steps[1]);
    }
    if (stepper3)
    {
        stepper3->setSpeedInHz(newSpeed);
        stepper3->setAcceleration(newAccel);
        stepper3->moveTo(steps[2]);
    }
    if (stepper4)
    {
        stepper4->setSpeedInHz(newSpeed);
        stepper4->setAcceleration(newAccel);
        stepper4->moveTo(steps[3]);
    }
    if (stepper5)
    {
        stepper5->setSpeedInHz(newSpeed);
        stepper5->setAcceleration(newAccel);
        stepper5->moveTo(steps[4]);
    }
    if (stepper6)
    {
        stepper6->setSpeedInHz(newSpeed);
        stepper6->setAcceleration(newAccel);
        stepper6->moveTo(steps[5]);
    }

    // Synchronization - wait until all motors have completed their movement
    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() ||
           stepper4->isRunning() || stepper5->isRunning() || stepper6->isRunning())
    {
        delay(10);
    }
}

float constrainPosition(float value)
{
    return max(-1.0f, min(1.0f, value));
}

void loadTrainingData(std::vector<TrainingData> &trainingData)
{
    File file = SD_MMC.open("/training_data.txt", FILE_READ);
    if (file)
    {
        while (file.available())
        {
            String line = file.readStringUntil('\n');
            TrainingData data;
            // Parse the line to fill the TrainingData structure
            int targetStart = line.indexOf("Target:") + 7;
            int targetEnd = line.indexOf(';', targetStart);
            String targetData = line.substring(targetStart, targetEnd);
            int solutionStart = line.indexOf("Solution:") + 9;
            int solutionEnd = line.indexOf(';', solutionStart);
            String solutionData = line.substring(solutionStart, solutionEnd);

            // Parse target data
            int commaIndex = 0;
            for (int i = 0; i < 7; ++i)
            {
                int nextComma = targetData.indexOf(',', commaIndex);
                if (nextComma != -1)
                {
                    if (i < 3)
                    {
                        ((float *)&data.targetX)[i] = targetData.substring(commaIndex, nextComma).toFloat();
                    }
                    else
                    {
                        ((float *)&data.quatW)[i - 3] = targetData.substring(commaIndex, nextComma).toFloat();
                    }
                    commaIndex = nextComma + 1;
                }
                else
                {
                    // Handle the last element
                    if (i < 3)
                    {
                        ((float *)&data.targetX)[i] = targetData.substring(commaIndex).toFloat();
                    }
                    else
                    {
                        ((float *)&data.quatW)[i - 3] = targetData.substring(commaIndex).toFloat();
                    }
                    break;
                }
            }

            // Parse solution data
            commaIndex = 0;
            for (int i = 0; i < 6; ++i)
            {
                int nextComma = solutionData.indexOf(',', commaIndex);
                if (nextComma != -1)
                {
                    ((float *)&data.joint1Angle)[i] = solutionData.substring(commaIndex, nextComma).toFloat();
                    commaIndex = nextComma + 1;
                }
                else
                {
                    // Handle the last element
                    ((float *)&data.joint1Angle)[i] = solutionData.substring(commaIndex).toFloat();
                    break;
                }
            }

            // Calculate current position if needed or set to target for simplicity
            data.currentX = data.targetX;
            data.currentY = data.targetY;
            data.currentZ = data.targetZ;

            trainingData.push_back(data);
        }
        file.close();
        Serial.print("Loaded ");
        Serial.print(trainingData.size());
        Serial.println(" training data entries.");
    }
    else
    {
        Serial.println("Failed to open training data file");
    }
}

void generateRotationQuaternion(float &w, float &x, float &y, float &z)
{
    static bool seeded = false;
    if (!seeded)
    {
        srand((unsigned int)time(NULL));
        seeded = true;
    }

    // Generate random rotation
    float angle = ((float)rand() / RAND_MAX) * 2 * M_PI;
    float axisX = ((float)rand() / RAND_MAX) * 2 - 1;   
    float axisY = ((float)rand() / RAND_MAX) * 2 - 1;
    float axisZ = ((float)rand() / RAND_MAX) * 2 - 1;
    float norm = sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    axisX /= norm;
    axisY /= norm;
    axisZ /= norm;

    // Create quaternion from angle and axis
    w = cos(angle / 2); 
    x = sin(angle / 2) * axisX;
    y = sin(angle / 2) * axisY;
    z = sin(angle / 2) * axisZ;

    Serial.print("Generated Rotation Quaternion: w=");
    Serial.print(w);
    Serial.print(", x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.print(y);
    Serial.print(", z=");
    Serial.println(z);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (!SD_MMC.begin())
    {
        Serial.println("Card Mount Failed");
        Serial.print("Card Type: ");
        Serial.println(SD_MMC.cardType());
        Serial.print("Card Size: ");
        Serial.println(SD_MMC.cardSize() / (1024.0 * 1024.0 * 1024.0), 2);
        Serial.println("Please format the SD card on a computer if this error persists.");

        return;
    }
    Serial.println("SD Card Mounted");
    Serial.print("Card Type: ");
    Serial.println(SD_MMC.cardType());
    Serial.print("Card Size: ");
    Serial.println(SD_MMC.cardSize() / (1024.0 * 1024.0 * 1024.0), 2);
    uint64_t totalSpace = SD_MMC.totalBytes();
    uint64_t usedSpace = SD_MMC.usedBytes();
    Serial.print("Free Space: ");
    Serial.print((totalSpace - usedSpace) / (1024.0 * 1024.0 * 1024.0), 2);
    Serial.println(" GB");

    // Training with real tracking data
    if (!trainWithRandomQuats)
    {
        BTSerial.begin("NECK_BT");
    }

    initWeights();

    if (areMotorsConnected)
    {
        engine.init();

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
        if (stepper1)
            stepper1->setCurrentPosition(0);
        if (stepper2)
            stepper2->setCurrentPosition(0);
        if (stepper3)
            stepper3->setCurrentPosition(0);
        if (stepper4)
            stepper4->setCurrentPosition(0);
        if (stepper5)
            stepper5->setCurrentPosition(0);
        if (stepper6)
            stepper6->setCurrentPosition(0);
    }
}

static std::vector<TrainingData> lastUpdates;

void loop()
{
    float quatW, quatX, quatY, quatZ;
    float targetX, targetY, targetZ;
    float currentX, currentY, currentZ;

    if (trainWithRandomQuats)
    {
        generateRotationQuaternion(quatW, quatX, quatY, quatZ);
        targetX = quatX;
        targetY = quatY;
        targetZ = quatZ;
    }
    else if (BTSerial.available())
    {
        String jsonString = BTSerial.readStringUntil('\n');

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, jsonString);

        if (error)
        {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
            return;
        }

        targetX = doc["x"];
        targetY = doc["y"];
        targetZ = doc["z"];
        quatW = doc["quaternion"][0];
        quatX = doc["quaternion"][1];
        quatY = doc["quaternion"][2];
        quatZ = doc["quaternion"][3];
    }

    float joint1Angle = previousJoint1Angle;
    float joint2Angle = previousJoint2Angle;
    float joint3Angle = previousJoint3Angle;
    float joint4Angle = previousJoint4Angle;
    float joint5Angle = previousJoint5Angle;
    float joint6Angle = previousJoint6Angle;

    float currentError = FLT_MAX;
    bool existingSolution = false;
    float bestError = FLT_MAX;
    size_t bestSolutionIndex = trainingData.size();

    // Check if we have an existing solution for these coordinates
    for (size_t i = 0; i < trainingData.size(); ++i)
    {
        const auto &solution = trainingData[i];
        if (solution.targetX == targetX && solution.targetY == targetY && solution.targetZ == targetZ &&
            solution.quatW == quatW && solution.quatX == quatX && solution.quatY == quatY && solution.quatZ == quatZ)
        {
            existingSolution = true;
            // Calculate error for this existing solution
            float tempX, tempY, tempZ;
            forwardKinematics(solution.joint1Angle, solution.joint2Angle, solution.joint3Angle,
                              solution.joint4Angle, solution.joint5Angle, solution.joint6Angle,
                              quatW, quatX, quatY, quatZ, tempX, tempY, tempZ);
            float error = calculateError(tempX, tempY, tempZ, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, quatW, quatX, quatY, quatZ);
            if (error < bestError)
            {
                bestError = error;
                bestSolutionIndex = i;
            }
        }
    }

    if (autoTrain)
    {
        // Use gradient descent to find potentially better solutions
        IKResult ikResult = gradientDescentIK(targetX, targetY, targetZ, joint1Angle, joint2Angle, joint3Angle,
                                              joint4Angle, joint5Angle, joint6Angle, trainingData, learningRate,
                                              quatW, quatX, quatY, quatZ);

        if (ikResult.solutionFound)
        {
            Serial.println("Solution found");
            Serial.print("Generated Target Position: ");
            Serial.print(targetX);
            Serial.print(", ");
            Serial.print(targetY);
            Serial.print(", ");
            Serial.println(targetZ);
            Serial.print("Joint Angles: ");
            Serial.print(joint1Angle);
            Serial.print(", ");
            Serial.print(joint2Angle);
            Serial.print(", ");
            Serial.print(joint3Angle);
            Serial.print(", ");
            Serial.print(joint4Angle);
            Serial.print(", ");
            Serial.print(joint5Angle);
            Serial.print(", ");
            Serial.println(joint6Angle);
            checkFreeHeap();
            Serial.print("Vector Size: ");
            Serial.println(trainingData.size());
            Serial.print("Vector Capacity: ");
            Serial.println(trainingData.capacity());

            // Use the error from gradientDescentIK
            currentError = ikResult.error;

            // Calculate current position
            forwardKinematics(joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle,
                              quatW, quatX, quatY, quatZ, currentX, currentY, currentZ);

            // Update model with the solution found
            unsigned int count = updateModel(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ,
                                             joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle);

            // Store the new solution in the last 25 updates with actual current position
            TrainingData newData = {targetX, targetY, targetZ,
                                    currentX, currentY, currentZ, // Use actual current position if available
                                    quatW, quatX, quatY, quatZ,
                                    joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle,
                                    currentError};
            lastUpdates.push_back(newData);

            // Save best solutions to SD card every 25 updates
            if (count % 25 == 0)
            {
                saveBestSolutions();
                lastUpdates.clear();
            }

            // If we found a solution with lower error than any existing solution or if no solution existed, update or add it
            if (currentError < bestError || !existingSolution)
            {
                if (existingSolution && bestSolutionIndex < trainingData.size())
                {
                    trainingData.erase(trainingData.begin() + bestSolutionIndex);
                }
                trainingData.push_back(newData);
            }

            previousJoint1Angle = joint1Angle;
            previousJoint2Angle = joint2Angle;
            previousJoint3Angle = joint3Angle;
            previousJoint4Angle = joint4Angle;
            previousJoint5Angle = joint5Angle;
            previousJoint6Angle = joint6Angle;
        }
        else
        {
            Serial.println("Generated position is out of reach.");
        }
    }
    else
    {
        // Use existing solution if available, otherwise predict
        if (existingSolution && bestSolutionIndex < trainingData.size())
        {
            const auto &bestSolution = trainingData[bestSolutionIndex];
        }
        else
        {
            // If no existing solution, predict angles
            predictAngles(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ,
                          joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle);
        }
    }

    if (areMotorsConnected && updateMotors)
    {
        // This function may require update
        moveHeadToPosition(joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle,
                           targetX, targetY, targetZ, quatW, quatX, quatY, quatZ);
    }

    delay(1000);
}

void saveBestSolutions()
{
    std::vector<TrainingData> bestSolutions;
    const size_t MAX_SOLUTIONS = 100; // Define based on your memory constraints

    Serial.print("Free Heap Before: ");
    Serial.println(ESP.getFreeHeap());

    File readFile = SD_MMC.open("/training_data.txt", FILE_READ);
    if (!readFile)
    {
        Serial.println("File does not exist, creating new file");
        readFile = SD_MMC.open("/training_data.txt", FILE_WRITE);
        if (readFile)
        {
            readFile.close();
        }
        else
        {
            Serial.println("Error creating file");
            return;
        }
        readFile = SD_MMC.open("/training_data.txt", FILE_READ);
        if (!readFile)
        {
            Serial.println("Error re-opening file for reading");
            return;
        }
    }

    if (readFile)
    {
        // Load all existing solutions
        while (readFile.available())
        {
            String line = readFile.readStringUntil('\n');
            Serial.println("Line: " + line);
            if (bestSolutions.size() < MAX_SOLUTIONS)
            {
                TrainingData data;
                if (parseLine(line, data))
                {
                    bestSolutions.push_back(data);
                }
                else
                {
                    Serial.println("Failed to parse line: " + line);
                }
            }
            else
            {
                Serial.println("Warning: Max solutions capacity reached, not adding more.");
                break;
            }
        }
        readFile.close();

        Serial.print("Number of Solutions Loaded: ");
        Serial.println(bestSolutions.size());

        // Merge new solutions with existing ones, keeping only the best
        for (const auto &newSolution : lastUpdates)
        {
            bool isBetter = true;
            float newError = newSolution.error; // Use the error from the structure

            for (auto &existingSolution : bestSolutions)
            {
                if (existingSolution.targetX == newSolution.targetX &&
                    existingSolution.targetY == newSolution.targetY &&
                    existingSolution.targetZ == newSolution.targetZ)
                {
                    float existingError = existingSolution.error; // Use stored error

                    if (newError < existingError)
                    {
                        existingSolution = newSolution; // Replace existing with new if it's better
                    }
                    isBetter = false;
                    break;
                }
            }
            if (isBetter && bestSolutions.size() < MAX_SOLUTIONS)
            {
                bestSolutions.push_back(newSolution);
            }
        }

        Serial.print("Number of Solutions After Merge: ");
        Serial.println(bestSolutions.size());

        // Print the most recent 25 solutions
        Serial.println("Recent 25 Solutions:");
        int count = 0;
        for (auto it = bestSolutions.rbegin(); it != bestSolutions.rend() && count < 25; ++it, ++count)
        {
            const auto &solution = *it;
            Serial.print("Target: ");
            Serial.print(solution.targetX);
            Serial.print(",");
            Serial.print(solution.targetY);
            Serial.print(",");
            Serial.print(solution.targetZ);
            Serial.print(",");
            Serial.print(solution.quatW);
            Serial.print(",");
            Serial.print(solution.quatX);
            Serial.print(",");
            Serial.print(solution.quatY);
            Serial.print(",");
            Serial.print(solution.quatZ);
            Serial.print(";");
            Serial.print("Solution: ");
            Serial.print(solution.joint1Angle);
            Serial.print(",");
            Serial.print(solution.joint2Angle);
            Serial.print(",");
            Serial.print(solution.joint3Angle);
            Serial.print(",");
            Serial.print(solution.joint4Angle);
            Serial.print(",");
            Serial.print(solution.joint5Angle);
            Serial.print(",");
            Serial.print(solution.joint6Angle);
            Serial.print(";");
            Serial.print("Error: ");
            Serial.println(solution.error);
        }

        // Delete or rename the old file, then write new data
        if (!SD_MMC.remove("/training_data.txt"))
        {
            Serial.println("Error removing old file");
            return;
        }

        File writeFile = SD_MMC.open("/training_data.txt", FILE_WRITE);
        if (writeFile)
        {
            for (const auto &solution : bestSolutions)
            {
                writeFile.print("Target: ");
                writeFile.print(solution.targetX);
                writeFile.print(",");
                writeFile.print(solution.targetY);
                writeFile.print(",");
                writeFile.print(solution.targetZ);
                writeFile.print(",");
                writeFile.print(solution.quatW);
                writeFile.print(",");
                writeFile.print(solution.quatX);
                writeFile.print(",");
                writeFile.print(solution.quatY);
                writeFile.print(",");
                writeFile.print(solution.quatZ);
                writeFile.print(";");
                writeFile.print("Solution: ");
                writeFile.print(solution.joint1Angle);
                writeFile.print(",");
                writeFile.print(solution.joint2Angle);
                writeFile.print(",");
                writeFile.print(solution.joint3Angle);
                writeFile.print(",");
                writeFile.print(solution.joint4Angle);
                writeFile.print(",");
                writeFile.print(solution.joint5Angle);
                writeFile.print(",");
                writeFile.print(solution.joint6Angle);
                writeFile.print(";");
                writeFile.print("Error: ");
                writeFile.print(solution.error);
                writeFile.println();
            }
            writeFile.close();
            Serial.println("Data successfully written to SD card");
        }
        else
        {
            Serial.println("Error opening file for writing");
        }

        Serial.print("Free Heap After: ");
        Serial.println(ESP.getFreeHeap());
    }
    else
    {
        Serial.println("Error opening file for reading after creation attempt");
    }
}

bool parseLine(String line, TrainingData &data)
{
    int targetStart = line.indexOf("Target:") + 7;
    int targetEnd = line.indexOf(';', targetStart);
    String targetData = line.substring(targetStart, targetEnd);
    int solutionStart = line.indexOf("Solution:") + 9;
    int errorStart = line.indexOf("Error:") + 6;

    // Parse target data
    int commaIndex = 0;
    for (int i = 0; i < 7; ++i)
    {
        int nextComma = targetData.indexOf(',', commaIndex);
        if (nextComma != -1)
        {
            if (i < 3)
            {
                ((float *)&data.targetX)[i] = targetData.substring(commaIndex, nextComma).toFloat();
            }
            else
            {
                ((float *)&data.quatW)[i - 3] = targetData.substring(commaIndex, nextComma).toFloat();
            }
            commaIndex = nextComma + 1;
        }
        else
        {
            if (i < 3)
            {
                ((float *)&data.targetX)[i] = targetData.substring(commaIndex).toFloat();
            }
            else
            {
                ((float *)&data.quatW)[i - 3] = targetData.substring(commaIndex).toFloat();
            }
            break;
        }
    }

    // Parse solution data
    commaIndex = 0;
    String solutionData = line.substring(solutionStart);
    for (int i = 0; i < 6; ++i)
    {
        int nextComma = solutionData.indexOf(',', commaIndex);
        if (nextComma != -1)
        {
            ((float *)&data.joint1Angle)[i] = solutionData.substring(commaIndex, nextComma).toFloat();
            commaIndex = nextComma + 1;
        }
        else
        {
            ((float *)&data.joint1Angle)[i] = solutionData.substring(commaIndex).toFloat();
            break;
        }
    }

    // Parse error
    if (errorStart != 5)
    {
        data.error = line.substring(errorStart).toFloat();
    }
    else
    {
        data.error = calculateError(data.currentX, data.currentY, data.currentZ,
                                    data.targetX, data.targetY, data.targetZ,
                                    data.quatW, data.quatX, data.quatY, data.quatZ,
                                    data.quatW, data.quatX, data.quatY, data.quatZ);
    }

    // For now, we assume current position equals target for simplicity
    data.currentX = data.targetX;
    data.currentY = data.targetY;
    data.currentZ = data.targetZ;

    return true;
}