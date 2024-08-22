#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "abcdef12-1234-1234-1234-abcdef123456"

// Define motor control pins
#define IN1 12 //IN1
#define IN2 14 //IN2
#define IN3 37 //IN3
#define IN4 26 //IN4

//int dataBLE;

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
      //dataBLE = value.toInt();
      //wheels
      //forward
      if (value == "forward") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      } 
      //left
      if (value == "left") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }
      //right
      if (value == "right") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
      }
      //reverse
      if (value == "reverse") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
      //motors off
      if (value == "park") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
      }
      //A button
      if (value == "a") {
        String response = "Button pressed: A";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }  
      //B button
      if (value == "b") {
        String response = "Button pressed: B";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //X button
      if (value == "x") {
        String response = "Button pressed: X";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();         
      } 
      //Y button
      if (value == "y") {
        String response = "Button pressed: Y";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();         
      } 
      //R1 button
      if (value == "r1") {
        String response = "Button pressed: R1";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();         
      } 
      //R2 button
      if (value == "r2") {        
        String response = "Button pressed: R2";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //L1 button
      if (value == "l1") {
        String response = "Button pressed: L1";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //L2 button
      if (value == "l2") {
        String response = "Button pressed: L2";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //D-pad Left
      if (value == "dpleft") {
        String response = "Button pressed: D-PAD LEFT";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //D-pad right
      if (value == "dpright") {
        String response = "Button pressed: D-PAD RIGHT";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //D-pad up
      if (value == "dpup") {
        String response = "Button pressed: D-PAD UP";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //D-pad down
      if (value == "dpdown") {
        String response = "Button pressed: D-PAD DOWN";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //Start
      if (value == "start") {
        String response = "Button pressed: START";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      }
      //Select
      if (value == "select") {
        String response = "Button pressed: SELECT";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      }           
    }
  }
};

void setup() {
  Serial.begin(115200);

  // Initialize BLE
  BLEDevice::init("v0_Robot");

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
