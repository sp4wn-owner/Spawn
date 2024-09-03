//ESP32 WROOM 32D
//Select ESP32 Dev Module in board manager
//CP210x USB driver is required to flash this board https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads
//To update driver go to device manager then find the usb driver then update by pointing to the folder you extracted
//Flash at 115200 Baud
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "abcdef12-1234-1234-1234-abcdef123456"
#define deviceName "v0_Robot"

// Define motor control pins
#define IN1 12 //IN1
#define IN2 14 //IN2
#define IN3 27 //IN3
#define IN4 26 //IN4

// Set up servos
const int MIN_VALUE = 0;
const int MAX_VALUE = 180;
Servo myServoPan;
Servo myServoTilt;
int servoPanPin = 18;
int servoTiltPin = 19; 
int panPosition;  // change this value to set the servo to a specific position
int tiltPosition;  // change this value to set the servo to a specific position

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String value;

// Callback class to handle client connections
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    resetBLE(pServer);
  }
  // Function to reset BLE (stop and start advertising)
  void resetBLE(BLEServer* pServer) {
    delay(500); // Short delay to ensure disconnection is complete
    pServer->startAdvertising(); // Restart advertising
  }
};

// Callback class to handle characteristic writes
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {      
      String response;
      response = "unknown command";
      //left joystick up
      if (value == "forward") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        response = "Moving forward";
      } 
      //left joystick left
      if (value == "left") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        response = "turning left";
      }
      //left joystick right
      if (value == "right") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        response = "Turning right";        
      }
      //left joystick down
      if (value == "reverse") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        response = "Reversing";
      }
      //left joystick center
      if (value == "park") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        response = "In park";
      }
      //right joystick up
      if (value == "rju") {
        tiltPosition += 5; //changes servo position by this number
        if (tiltPosition > MAX_VALUE) {
          tiltPosition = 180;
        }
        if (tiltPosition < MIN_VALUE) {
          tiltPosition = 0;
        }
        myServoTilt.write(tiltPosition);
        response = "tilt servo position: " + tiltPosition;
      }
      //right joystick left
      if (value == "rjl") {
        panPosition -= 5; //changes servo position by this number
        if (panPosition > MAX_VALUE) {
          panPosition = 180;
        }
        if (panPosition < MIN_VALUE) {
          panPosition = 0;
        }
        myServoPan.write(panPosition);
        response = "pan servo position: " + panPosition;
      }
      //right joystick right
      if (value == "rjr") {
        panPosition += 5; //changes servo position by this number
        if (panPosition > MAX_VALUE) {
          panPosition = 180;
        }
        if (panPosition < MIN_VALUE) {
          panPosition = 0;
        }
        myServoPan.write(panPosition);
        response = "pan servo position: " + panPosition;
      }
      //right joystick down
      if (value == "rjd") {
        tiltPosition -= 5; //changes servo position by this number
        if (tiltPosition > MAX_VALUE) {
          tiltPosition = 180;
        }
        if (tiltPosition < MIN_VALUE) {
          tiltPosition = 0;
        }
        myServoTilt.write(tiltPosition);
        response = "tilt servo position: " + tiltPosition;
      }
      //right joystick center
      if (value == "rjc") {
        response = "right stick center";
      }
      //A button
      if (value == "0") {
        response = "Button pressed: A";
      }  
      //B button
      if (value == "1") {
        response = "Button pressed: B";       
      } 
      //X button
      if (value == "2") {
        response = "Button pressed: X";        
      } 
      //Y button
      if (value == "3") {
        response = "Button pressed: Y";        
      } 
      //L1 button
      if (value == "4") {
        response = "Button pressed: L1";        
      } 
      //R1 button
      if (value == "5") {        
        response = "Button pressed: R1";
        }      //L2 button
      if (value == "6") {
        response = "Button pressed: L2";      
      } 
      //R2 button
      if (value == "7") {
        response = "Button pressed: R2";       
      } 
      //Select
      if (value == "8") {
        response = "Button pressed: SELECT";       
      } 
      //Start
      if (value == "9") {
        response = "Button pressed: START";     
      } 
      //Left Joystick Pressed
      if (value == "10") {
        response = "Button pressed: Left Stick";       
      } 
      //Right Joystick Pressed
      if (value == "11") {
        response = "Button pressed: Right Stick";     
      } 
      //D-pad up
      if (value == "12") {
        response = "Button pressed: D-pad UP";      
      }
      //D-pad down
      if (value == "13") {
        response = "Button pressed: D-pad DOWN";  
      } 
      //D-pad left
      if (value == "14") {
        response = "Button pressed: D-pad LEFT";     
      }
      //D-pad right
      if (value == "15") {
        response = "Button pressed: D-pad RIGHT";      
      }          
      pCharacteristic->setValue(response);
      pCharacteristic->notify();
    }
  }
};

void setup() {

  myServoPan.attach(servoPanPin);
  myServoTilt.attach(servoTiltPin);
  Serial.begin(115200);
  panPosition = 90; //centers servo when initialized
  tiltPosition = 90;
  myServoPan.write(panPosition);
  myServoTilt.write(tiltPosition);

  // Initialize BLE
  BLEDevice::init(deviceName);

  // Create a BLE server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create a BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,                     
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE |
                     BLECharacteristic::PROPERTY_NOTIFY
                   );

  pCharacteristic->addDescriptor(new BLE2902());  // Needed for NOTIFY
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
}
