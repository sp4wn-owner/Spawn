// TESTING WITH RANDOM QUATERNIONS
#include <cmath>
#include <float.h>
#include <ArduinoJson.h>
#include <vector>
#include <cstdlib>
#include <ctime>

const float link1Length = 10.0;
const float link2Length = 10.0;
const float link3Length = 10.0;

// Assuming these scales relate to how the motors should react to quaternion components
const float qxScale = 10.0;    // Adjust as needed for X-axis rotation
const float qyScale = 10.0;    // Adjust as needed for Y-axis rotation
const float qzScale = 10.0;    // Adjust as needed for Z-axis rotation
const float qwScale = 10.0;    // Adjust as needed for the scalar part of the quaternion
const float heightScale = 400.0; // Adjust as needed, based on the mechanics of your platform

// New scaling constants
const float standardScale = 0.1;   // Adjust as needed
const float pitchScale = 0.1;   // Adjust as needed
const float rollScale = 0.1;    // Adjust as needed
const float yawScale = 0.1;     // Adjust as needed for Z-axis movement
const float rollMovementScale = 10.0; // Adjust as needed for roll movement
const float pitchMovementScale = 10.0; // Adjust as needed for pitch movement

float previousJoint1Angle = 0.0;
float previousJoint2Angle = 0.0;
float previousJoint3Angle = 0.0;
float previousJoint4Angle = 0.0;
float previousJoint5Angle = 0.0;
float previousJoint6Angle = 0.0;

struct TrainingData {
    float targetX, targetY, targetZ;
    float quatW, quatX, quatY, quatZ;
    float joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle;
    float currentX, currentY, currentZ;
};

// Function prototypes can be here to avoid "implicit declaration" warnings
void initWeights();
void predictAngles(float, float, float, float, float, float, float, float&, float&, float&, float&, float&, float&);
bool gradientDescentIK(float, float, float, float&, float&, float&, float&, float&, float&, std::vector<TrainingData>&, float&, float, float, float, float);
void updateModel(float, float, float, float, float, float, float, float, float, float, float, float, float);
void quaternionToRotationMatrix(float, float, float, float, float[3][3]);
void forwardKinematics(float, float, float, float, float, float, float, float&, float&, float&);
float calculateError(float, float, float, float, float, float, float, float, float, float);
void computeGradients(float, float, float, float, float, float, float, float, float, float, float&, float&, float&);
void generatePositionalQuaternion(float&, float&, float&, float&);

// Simple model parameters for linear regression
float weights[6][7]; // 6 joint angles, each predicted by 7 inputs
float learningRate = 0.1;

void initWeights() {
    // Seed the random number generator once
    static bool seeded = false;
    if (!seeded) {
        srand(static_cast<unsigned int>(time(NULL)));  // Use C-style cast here since static_cast isn't working
        seeded = true;
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 7; ++j) {
            // Generate a small random value between -0.05 and 0.05
            weights[i][j] = ((float)rand() / RAND_MAX) * 0.1 - 0.05;
        }
    }
}

// Predict joint angles from the current model
void predictAngles(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ, 
                   float &joint1Angle, float &joint2Angle, float &joint3Angle, float &joint4Angle, float &joint5Angle, float &joint6Angle) {
    float inputs[7] = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ};
    
    for (int i = 0; i < 6; ++i) { // Loop through 6 joints
        float predictionInRadians = 0.0;
        for (int j = 0; j < 7; j++) {
            predictionInRadians += weights[i][j] * inputs[j];
        }
        
        // Convert prediction from radians to degrees and apply scale
        float predictionInDegrees = predictionInRadians * (180.0 / M_PI);
        
        if (i == 0) joint1Angle = predictionInDegrees * pitchScale;
        else if (i == 1) joint2Angle = predictionInDegrees * rollScale;
        else if (i == 2) joint3Angle = predictionInDegrees * yawScale;
        else if (i == 3) joint4Angle = predictionInDegrees * standardScale; // Add new scale for joint 4
        else if (i == 4) joint5Angle = predictionInDegrees * standardScale; // Add new scale for joint 5
        else joint6Angle = predictionInDegrees * standardScale; // Add new scale for joint 6
    }
}

// Update model with new data (session learning)
// Update model with new data (session learning)
void updateModel(float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ, 
                 float joint1Angle, float joint2Angle, float joint3Angle, float joint4Angle, float joint5Angle, float joint6Angle) {
    static unsigned int updateCount = 0; // Static counter for model updates
    updateCount++; // Increment each time updateModel is called

    Serial.println("Updating Model:");
    Serial.print("Model Update Count: "); Serial.println(updateCount); // Print the update count
    Serial.print("Target Position: ");
    Serial.print(targetX); Serial.print(", "); Serial.print(targetY); Serial.print(", "); Serial.println(targetZ);
    Serial.print("Quaternion: ");
    Serial.print(quatW); Serial.print(", "); 
    Serial.print(quatX); Serial.print(", "); 
    Serial.print(quatY); Serial.print(", "); 
    Serial.println(quatZ);
    Serial.print("Joint Angles: ");
    Serial.print(joint1Angle); Serial.print(", "); 
    Serial.print(joint2Angle); Serial.print(", "); 
    Serial.print(joint3Angle); Serial.print(", "); 
    Serial.print(joint4Angle); Serial.print(", "); 
    Serial.print(joint5Angle); Serial.print(", "); 
    Serial.println(joint6Angle);

    float inputs[7] = {targetX, targetY, targetZ, quatW, quatX, quatY, quatZ};
    float outputs[6] = {
        joint1Angle / (pitchScale * (180.0 / M_PI)), 
        joint2Angle / (rollScale * (180.0 / M_PI)), 
        joint3Angle / (yawScale * (180.0 / M_PI)),
        joint4Angle / (standardScale * (180.0 / M_PI)), // Convert new joints back to radians
        joint5Angle / (standardScale * (180.0 / M_PI)),
        joint6Angle / (standardScale * (180.0 / M_PI))
    };
    
    for (int i = 0; i < 6; ++i) { // for each joint angle
        float prediction = 0;
        for (int j = 0; j < 7; j++) {
            prediction += weights[i][j] * inputs[j];
        }
        float error = outputs[i] - prediction;
        
        Serial.print("Joint "); Serial.print(i+1); Serial.print(" - Prediction: ");
        Serial.print(prediction); Serial.print(", Actual: ");
        Serial.print(outputs[i]); Serial.print(", Error: ");
        Serial.println(error);
        
        for (int j = 0; j < 7; j++) {
            weights[i][j] += learningRate * error * inputs[j]; // Gradient descent update rule
        }
    }
}

void quaternionToRotationMatrix(float quatW, float quatX, float quatY, float quatZ, float rotMat[3][3]) {
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

// Function to calculate forward kinematics
void forwardKinematics(float joint1Angle, float joint2Angle, float joint3Angle, float joint4Angle, float joint5Angle, float joint6Angle, 
                       float quatW, float quatX, float quatY, float quatZ, float &x, float &y, float &z) {
    // Simplified 6DoF forward kinematics for a serial manipulator
    // This is a placeholder; actual calculation depends on robot geometry
    float posX = link1Length * cos(joint1Angle) + 
                 link2Length * cos(joint1Angle + joint2Angle) + 
                 link3Length * cos(joint1Angle + joint2Angle + joint3Angle) +
                 link3Length * cos(joint1Angle + joint2Angle + joint3Angle + joint4Angle); // Assume joint4 affects this plane

    float posY = link1Length * sin(joint1Angle) + 
                 link2Length * sin(joint1Angle + joint2Angle) + 
                 link3Length * sin(joint1Angle + joint2Angle + joint3Angle) +
                 link3Length * sin(joint1Angle + joint2Angle + joint3Angle + joint4Angle); // Same assumption

    float posZ = link3Length * sin(joint3Angle) + 
                 link3Length * sin(joint3Angle + joint4Angle) + 
                 link3Length * sin(joint5Angle) + 
                 link3Length * sin(joint6Angle); // Joint 5 and 6 might control additional rotations

    float rotMat[3][3];
    quaternionToRotationMatrix(quatW, quatX, quatY, quatZ, rotMat);

    // Apply rotation using the matrix
    x = rotMat[0][0] * posX + rotMat[0][1] * posY + rotMat[0][2] * posZ;
    y = rotMat[1][0] * posX + rotMat[1][1] * posY + rotMat[1][2] * posZ;
    z = rotMat[2][0] * posX + rotMat[2][1] * posY + rotMat[2][2] * posZ;
}

// Error function
float calculateError(float currentX, float currentY, float currentZ, float targetX, float targetY, float targetZ, 
                     float quatW, float quatX, float quatY, float quatZ) {
    float posError = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2) + pow(targetZ - currentZ, 2));
    // This is a simplified orientation error calculation; might need refinement
    float oriError = 1 - abs(quatW * quatW + quatX * quatX + quatY * quatY + quatZ * quatZ); // This might not be correct for orientation error, but it's a start
    return posError + oriError; // Combine both errors
}

// Compute analytical gradients for efficiency
void computeGradients(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, 
                      float targetX, float targetY, float targetZ, float quatW, float quatX, float quatY, float quatZ, 
                      float &grad1, float &grad2, float &grad3, float &grad4, float &grad5, float &grad6) {
    float epsilon = 0.001;
    float currentX, currentY, currentZ;
    float x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5, y5, z5, x6, y6, z6;

    forwardKinematics(joint1, joint2, joint3, joint4, joint5, joint6, quatW, quatX, quatY, quatZ, currentX, currentY, currentZ);

    // Gradient for each joint
    for (int i = 0; i < 6; i++) {
        float angles[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
        angles[i] += epsilon; // Perturb one joint at a time
        
        forwardKinematics(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], 
                          quatW, quatX, quatY, quatZ, x1, y1, z1);
        
        float error = calculateError(x1, y1, z1, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ);
        float baseError = calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ);
        
        float* gradPtr = &grad1 + i; // Pointer arithmetic to get the right gradient
        *gradPtr = (error - baseError) / epsilon;
    }
}

// Gradient descent IK solver with improved convergence
bool gradientDescentIK(float targetX, float targetY, float targetZ, 
                       float &joint1Angle, float &joint2Angle, float &joint3Angle, 
                       float &joint4Angle, float &joint5Angle, float &joint6Angle, 
                       std::vector<TrainingData> &data, float &learningRate, 
                       float quatW, float quatX, float quatY, float quatZ) {
    const int maxIterations = 1000;
    float error = FLT_MAX;
    float previousError = FLT_MAX;
    bool solutionFound = false;

    Serial.println("Starting Gradient Descent IK for 6DoF:");
    Serial.print("Target Position: ");
    Serial.print(targetX); Serial.print(", "); Serial.print(targetY); Serial.print(", "); Serial.println(targetZ);

    float jointAngles[6] = {joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle};

    for (int i = 0; i < maxIterations; i++) {
        float currentX, currentY, currentZ;
        forwardKinematics(jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5], 
                          quatW, quatX, quatY, quatZ, currentX, currentY, currentZ);

        // Constrain current position to [-1, 1]
        currentX = constrainPosition(currentX);
        currentY = constrainPosition(currentY);
        currentZ = constrainPosition(currentZ);

        error = calculateError(currentX, currentY, currentZ, targetX, targetY, targetZ, quatW, quatX, quatY, quatZ);

        if (error < 1) {  // Error threshold set to 1
          Serial.println("Solution found within error threshold.");
          Serial.print("Error Rate: "); 
          Serial.println(error);  // Print the error rate
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

        // Update joint angles with gradients
        float scales[6] = {pitchScale, rollScale, yawScale, 0.1, 0.1, 0.1}; // Example scales for each joint
        for (int j = 0; j < 6; j++) {
            jointAngles[j] -= learningRate * gradients[j] * scales[j];
        }

        // Learning rate adjustment
        if (i > 5 && error > previousError) {
            learningRate *= 0.9; // Decrease learning rate if error increases
        } else if (error < previousError) {
            learningRate = min(learningRate * 1.05, 0.1); // Increase learning rate but cap it
        }
        previousError = error;
    }

    if (!solutionFound) {
        float currentX, currentY, currentZ; 
        forwardKinematics(jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5], 
                          quatW, quatX, quatY, quatZ, currentX, currentY, currentZ); 
        Serial.println("No solution found after max iterations.");
        Serial.print("Final Error: "); Serial.println(error);
        Serial.print("Final Position: ");
        Serial.print(currentX); Serial.print(", "); Serial.print(currentY); Serial.print(", "); Serial.println(currentZ);
        Serial.print("Final Joint Angles: ");
        for (int i = 0; i < 6; i++) {
            Serial.print(jointAngles[i]); 
            if (i < 5) Serial.print(", ");
        }
        Serial.println();
    } else {
        // Convert to motor steps for all 6 motors
        const float DISTANCE_PER_STEP = ((2.0 / 6400) / 8) * 100; // From your motor setup, in degrees per step
        
        int steps[6];
        float scales[6] = {pitchScale, rollScale, yawScale, 0.1, 0.1, 0.1}; // Declare scales here
        for (int i = 0; i < 6; i++) {
            steps[i] = (jointAngles[i] * (180.0 / M_PI) * scales[i]) / DISTANCE_PER_STEP;
        }

        // Define motor limits here (example values)
        int maxSteps = 10000; // Example, adjust based on your hardware
        int minSteps = -10000; // Example, adjust based on your hardware

        for (int i = 0; i < 6; i++) {
            steps[i] = constrain(steps[i], minSteps, maxSteps);
            Serial.print("Steps for Motor "); Serial.print(i+1); Serial.print(": "); Serial.println(steps[i]);
        }

        // Here you can call your motor control function with these step values
        // Example:
        // moveHead(steps[0], steps[1], steps[2], steps[3], steps[4], steps[5], 0, 1.0, 1.0, 0, 0);
    }

    // Update joint angles back to the passed references
    joint1Angle = jointAngles[0];
    joint2Angle = jointAngles[1];
    joint3Angle = jointAngles[2];
    joint4Angle = jointAngles[3];
    joint5Angle = jointAngles[4];
    joint6Angle = jointAngles[5];

    return solutionFound;
}

void generatePositionalQuaternion(float &w, float &x, float &y, float &z) {
    static bool seeded = false;
    if (!seeded) {
        srand((unsigned int)time(NULL)); // Seed only once
        seeded = true;
    }

    // Generate random position within a range
    const float maxPosition = 10.0; // Adjust according to your workspace size
    x = ((float)rand() / RAND_MAX) * maxPosition * 2 - maxPosition; // -max to max for position
    y = ((float)rand() / RAND_MAX) * maxPosition * 2 - maxPosition;
    z = ((float)rand() / RAND_MAX) * maxPosition * 2 - maxPosition;

    // Generate random rotation
    float angle = ((float)rand() / RAND_MAX) * 2 * PI; // Random angle for rotation
    float axisX = ((float)rand() / RAND_MAX) * 2 - 1; // Random axis components
    float axisY = ((float)rand() / RAND_MAX) * 2 - 1;
    float axisZ = ((float)rand() / RAND_MAX) * 2 - 1;
    float norm = sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);
    axisX /= norm; axisY /= norm; axisZ /= norm; // Normalize axis

    // Combine rotation with position
    w = cos(angle/2); // Rotation scalar
    x += sin(angle/2) * axisX; // Add rotation to position
    y += sin(angle/2) * axisY;
    z += sin(angle/2) * axisZ;

    // Normalize to ensure it's a unit quaternion (for rotation purposes)
    float fullNorm = sqrt(w*w + x*x + y*y + z*z);
    w /= fullNorm;
    x /= fullNorm;
    y /= fullNorm;
    z /= fullNorm;

    Serial.print("Generated Positional Quaternion: w=");
    Serial.print(w); Serial.print(", x="); Serial.print(x); Serial.print(", y="); 
    Serial.print(y); Serial.print(", z="); Serial.println(z);
}

float constrainPosition(float value) {
    return max(-1.0f, min(1.0f, value));
}

void setup() {
    Serial.begin(115200);
    initWeights(); // Initialize model weights
}

std::vector<TrainingData> trainingData;

void loop() {
    float quatW, quatX, quatY, quatZ;
    generatePositionalQuaternion(quatW, quatX, quatY, quatZ);

    // Constrain the position to [-1, 1]
    float targetX = constrainPosition(quatX);
    float targetY = constrainPosition(quatY);
    float targetZ = constrainPosition(quatZ);

    float joint1Angle = previousJoint1Angle;
    float joint2Angle = previousJoint2Angle;
    float joint3Angle = previousJoint3Angle;
    float joint4Angle = previousJoint4Angle;
    float joint5Angle = previousJoint5Angle;
    float joint6Angle = previousJoint6Angle;
    
    // Predict angles based on the generated data
    predictAngles(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, 
                  joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle);

    // Use gradient descent with the generated data as the target
    if (gradientDescentIK(targetX, targetY, targetZ, joint1Angle, joint2Angle, joint3Angle, 
                          joint4Angle, joint5Angle, joint6Angle, trainingData, learningRate, 
                          quatW, quatX, quatY, quatZ)) {
        // Update model with the solution found
        updateModel(targetX, targetY, targetZ, quatW, quatX, quatY, quatZ, 
                    joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle, joint6Angle);

        previousJoint1Angle = joint1Angle;
        previousJoint2Angle = joint2Angle;
        previousJoint3Angle = joint3Angle;
        previousJoint4Angle = joint4Angle;
        previousJoint5Angle = joint5Angle;
        previousJoint6Angle = joint6Angle;

        Serial.println("Solution found");
        Serial.print("Generated Target Position: ");
        Serial.print(targetX); Serial.print(", "); Serial.print(targetY); Serial.print(", "); Serial.println(targetZ);
        Serial.print("Joint Angles: ");
        Serial.print(joint1Angle); Serial.print(", "); 
        Serial.print(joint2Angle); Serial.print(", "); 
        Serial.print(joint3Angle); Serial.print(", "); 
        Serial.print(joint4Angle); Serial.print(", "); 
        Serial.print(joint5Angle); Serial.print(", "); 
        Serial.println(joint6Angle);

        // Convert to motor steps and control motors here
        // ...
    } else {
        Serial.println("Generated position is out of reach.");
    }

    delay(1000);
}