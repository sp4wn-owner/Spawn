#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "abcdef12-1234-1234-1234-abcdef123456"

// Define motor control pins
#define IN1 14 //IN1
#define IN2 15 //IN2
#define IN3 2 //IN3
#define IN4 4 //IN4


Servo myServoPan;  // create servo object to control a servo
Servo myServoTilt;  // create servo object to control a servo

int servoPanPin = 12;  // GPIO pin connected to the servo
int servoTiltPin = 13;  // GPIO pin connected to the servo
int dataBLE;
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
      //servos
      dataBLE = value.toInt();
      if (dataBLE < 1999) {
        if (dataBLE > 999) {
          tiltPosition = dataBLE - 1000;
          myServoTilt.write(tiltPosition);
          } else {
          panPosition = dataBLE;
          myServoPan.write(panPosition);
          }  
      } else {
        //wheels
        //forward
        if (dataBLE == 2000) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
        } 
        //left
        else if (dataBLE == 2001) {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
        }
        //right
        else if (dataBLE == 2002) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, LOW);
        }
        //reverse
        else if (dataBLE == 2003) {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
        }
        //motors off
        else if (dataBLE == 2004) {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, LOW);
        }
      }
    }
  }
};

void setup() {
  myServoPan.attach(servoPanPin);  // attaches the servo on GPIO pin 2 to the servo object
  myServoTilt.attach(servoTiltPin);  // attaches the servo on GPIO pin 4 to the servo object
  Serial.begin(115200);
  panPosition = 90;
  tiltPosition = 90;

  // Initialize BLE
  BLEDevice::init("ESP32_BLE_Server");

  // Create a BLE server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create a BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE
                   );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  //Serial.println("Waiting for a client connection to notify...");

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  myServoPan.write(panPosition);
  myServoTilt.write(tiltPosition);
}

void loop() {
}
