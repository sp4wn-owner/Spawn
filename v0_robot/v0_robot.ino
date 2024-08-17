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

int dataBLE;

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
      dataBLE = value.toInt();
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
  Serial.begin(115200);

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
}

void loop() {
}
