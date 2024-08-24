//ESP32 WROOM 32D
//Select ESP32 Dev Module in board manager
//CP210x USB driver is required to flash this board https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads
//To update driver go to device manager then find the usb driver then update by pointing to the folder you extracted
//Flash at 115200 Baud
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
#define IN3 27 //IN3
#define IN4 26 //IN4

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
      //left joystick up
      if (value == "forward") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        String response = "Moving forward";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      } 
      //left joystick left
      if (value == "left") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        String response = "turning left";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //left joystick right
      if (value == "right") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        String response = "Turning right";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //left joystick down
      if (value == "reverse") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        String response = "Reversing";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //left joystick center
      if (value == "park") {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        String response = "In park";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //right joystick up
      if (value == "rju") {
        String response = "right stick up";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //right joystick left
      if (value == "rjl") {
        String response = "right stick left";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //right joystick right
      if (value == "rjr") {
        String response = "right stick right";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //right joystick down
      if (value == "rjd") {
        String response = "right stick down";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //right joystick center
      if (value == "rjc") {
        String response = "right stick center";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }
      //A button
      if (value == "0") {
        String response = "Button pressed: A";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
      }  
      //B button
      if (value == "1") {
        String response = "Button pressed: B";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //X button
      if (value == "2") {
        String response = "Button pressed: X";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();         
      } 
      //Y button
      if (value == "3") {
        String response = "Button pressed: Y";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();         
      } 
      //L1 button
      if (value == "4") {
        String response = "Button pressed: L1";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();         
      } 
      //R1 button
      if (value == "5") {        
        String response = "Button pressed: R1";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //L2 button
      if (value == "6") {
        String response = "Button pressed: L2";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //R2 button
      if (value == "7") {
        String response = "Button pressed: R2";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //Select
      if (value == "8") {
        String response = "Button pressed: SELECT";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //Start
      if (value == "9") {
        String response = "Button pressed: START";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //Left Joystick Pressed
      if (value == "10") {
        String response = "Button pressed: Left Stick";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //Right Joystick Pressed
      if (value == "11") {
        String response = "Button pressed: Right Stick";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //D-pad up
      if (value == "12") {
        String response = "Button pressed: D-pad UP";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      }
      //D-pad down
      if (value == "13") {
        String response = "Button pressed: D-pad DOWN";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      } 
      //D-pad left
      if (value == "14") {
        String response = "Button pressed: D-pad LEFT";
        pCharacteristic->setValue(response);
        pCharacteristic->notify();        
      }
      //D-pad right
      if (value == "15") {
        String response = "Button pressed: D-pad RIGHT";
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
