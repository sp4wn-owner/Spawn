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
#include <ArduinoJson.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "abcdef12-1234-1234-1234-abcdef123456"
#define deviceName "v0_Robot"

// GPIO pins to control
const int gpioPins[] = {2,4,5,12,13,14,15,18,19,21,22,23,26,27};
const int numPins = sizeof(gpioPins) / sizeof(gpioPins[0]);

// Define motor control pins
#define IN1 12 //IN1
#define IN2 13 //IN2
#define IN3 27 //IN3
#define IN4 26 //IN4

// Set up servos
const int MIN_VALUE = 0;
const int MAX_VALUE = 180;
Servo myServoPan;
Servo myServoTilt;
int servoPanPin = 25;
int servoTiltPin = 33; 
bool handlingCMD = false;
const int deadZone = 10; // Adjust as needed
int tiltPosition = 90; // Initial servo position
int panPosition = 90; // Initial servo position

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String value;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    for (int i = 0; i < numPins; i++) {
        digitalWrite(gpioPins[i], LOW);
    }
    deviceConnected = false;
    resetBLE(pServer);
  }

  void resetBLE(BLEServer* pServer) {
    delay(500); 
    pServer->startAdvertising(); 
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    Serial.print(F("received cmd"));
    if (!handlingCMD) {
      handlingCMD = true;
      std::string rawValue = std::string(pCharacteristic->getValue().c_str());
      if (rawValue.length() > 0) {
        String response = "unknown command";
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, rawValue);

        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        }

        if (doc.containsKey("joystickSelector") && doc.containsKey("joystickX") && doc.containsKey("joystickY") && doc.containsKey("buttons")) {
          const char* joystickSelector = doc["joystickSelector"];
          float joystickX = doc["joystickX"];
          float joystickY = doc["joystickY"];
          JsonArray buttons = doc["buttons"];

          if (strcmp(joystickSelector, "joystick1") == 0 || strcmp(joystickSelector, "joystick3") == 0) {
            handleJoystickCommands(joystickX, joystickY, response);
          }

          if (strcmp(joystickSelector, "joystick2") == 0 || strcmp(joystickSelector, "joystick4") == 0) {
            handleServoCommands(joystickX, joystickY, response);
          }

          if (!buttons.isNull()) {
            handleButtonCommands(buttons, response);
          }

          Serial.println(response);
        } else {
          Serial.println(F("The JSON object does not have the required fields"));
        }
        pCharacteristic->setValue(response);
        pCharacteristic->notify();
        handlingCMD = false;
      }
    }
  }

  void handleJoystickCommands(float joystickX, float joystickY, String &response) {
    String command;

    if (joystickY > 0.5) {
      command = "reverse";
    } else if (joystickY < -0.5) {
      command = "forward";
    } else if (joystickX > 0.5) {
      command = "right";
    } else if (joystickX < -0.5) {
      command = "left";
    } else if (abs(joystickX) < deadZone && abs(joystickY) < deadZone) {
      command = "park";
    }

    if (command == "forward") {
      moveForward();
      response = "Moving forward";
    } else if (command == "left") {
      turnLeft();
      response = "Turning left";
    } else if (command == "right") {
      turnRight();
      response = "Turning right";
    } else if (command == "reverse") {
      reverse();
      response = "Reversing";
    } else if (command == "park") {
      park();
      response = "In park";
    } else {
      response = "Unknown joystick command";
    }
  }

  void handleServoCommands(float joystickX, float joystickY, String &response) {
    String servoCommand;

    if (joystickY > 0.5) {
      servoCommand = "down";
    } else if (joystickY < -0.5) {
      servoCommand = "up";
    } else if (joystickX > 0.5) {
      servoCommand = "right";
    } else if (joystickX < -0.5) {
      servoCommand = "left";
    } else if (abs(joystickX) < deadZone && abs(joystickY) < deadZone) {
      servoCommand = "neutral";
    }

    if (servoCommand == "up") {
      tiltPosition = min(MAX_VALUE, tiltPosition + 5);
      response = "Servo: Moving up";
    } else if (servoCommand == "down") {
      tiltPosition = max(MIN_VALUE, tiltPosition - 5);
      response = "Servo: Moving down";
    } else if (servoCommand == "right") {
      panPosition = min(MAX_VALUE, panPosition + 5);
      response = "Servo: Moving right";
    } else if (servoCommand == "left") {
      panPosition = max(MIN_VALUE, panPosition - 5);
      response = "Servo: Moving left";
    } else if (servoCommand == "neutral") {
      response = "Servo: Neutral position";
    } else {
      response = "Servo: unknown command";
    }
  }

  void handleButtonCommands(JsonArray buttons, String &response) {
    for (JsonVariant button : buttons) {
      if (button == "0") {
        response = "Button pressed: A";
      } else if (button == "1") {
        response = "Button pressed: B";
      } else if (button == "2") {
        response = "Button pressed: X";
      } else if (button == "3") {
        response = "Button pressed: Y";
      } else if (button == "4") {
        response = "Button pressed: L1";
      } else if (button == "5") {
        response = "Button pressed: R1";
      } else if (button == "6") {
        response = "Button pressed: L2";
      } else if (button == "7") {
        response = "Button pressed: R2";
      } else if (button == "8") {
        response = "Button pressed: SELECT";
      } else if (button == "9") {
        response = "Button pressed: START";
      } else if (button == "10") {
        response = "Button pressed: Left Stick";
      } else if (button == "11") {
        response = "Button pressed: Right Stick";
      } else if (button == "12") {
        moveForward();
        response = "Moving forward";
      } else if (button == "13") {
        reverse();
        response = "Reversing";
      } else if (button == "14") {
        turnLeft();
        response = "Turning left";
      } else if (button == "15") {
        turnRight();
        response = "Turning right";
      } else if (button == "16") {
        park();
        response = "In park";
      } else if (button == "off") {
        response = "Pins off";
      } else {
        response = "unknown button command";
      }
    }
  }

  void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  void reverse() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  void park() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
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

  pService->start();

  pServer->getAdvertising()->start();

  // Initialize GPIO pins
    for (int i = 0; i < numPins; i++) {
        pinMode(gpioPins[i], OUTPUT);
    }
}

void loop() {
}
