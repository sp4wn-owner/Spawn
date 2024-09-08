//ESP32 WROOM 32D
//Select ESP32 Dev Module in board manager
//CP210x USB driver is required to flash this board https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads
//To update driver go to device manager then find the usb driver then update by pointing to the folder you extracted
//Flash at 115200 Baud
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// Replace these with your network credentials
const char* ssid = "";
const char* password = "";

// Set the hostname
const char* hostname = "ESP32WebServer";

WebSocketsServer webSocket = WebSocketsServer(81);

// Define motor control pins
#define IN12 12
#define IN14 13
#define IN27 27
#define IN26 26

// Define servo pins
#define IN25 25
#define IN33 33

// Set up servos
const int MIN_VALUE = 0;
const int MAX_VALUE = 180;
Servo myServoPan;
Servo myServoTilt;
int servoPanPin = IN25;
int servoTiltPin = IN33; 
int panPosition;
int tiltPosition;

void setup() {
  myServoPan.attach(servoPanPin);
  myServoTilt.attach(servoTiltPin);
  Serial.begin(115200);
  panPosition = 90; //centers servo when initialized
  tiltPosition = 90;
  myServoPan.write(panPosition);
  myServoTilt.write(tiltPosition);

  // Set the hostname
  WiFi.setHostname(hostname);

  // Set motor control pins as outputs
  pinMode(IN12, OUTPUT);
  pinMode(IN14, OUTPUT);
  pinMode(IN27, OUTPUT);
  pinMode(IN26, OUTPUT);
  
  // Initialize GPIO pins
  //pinMode(IN2, OUTPUT);
  //pinMode(IN4, OUTPUT);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
}

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    String value = String((char*)payload);
    String response;
    // Default response if no valid command is received
    response = "unknown command";

    if (value.length() > 0) { 
      //left joystick up
      if (value == "forward") {
        digitalWrite(IN12, HIGH);
        digitalWrite(IN14, LOW);
        digitalWrite(IN27, HIGH);
        digitalWrite(IN26, LOW);
        response = "moving forward";
      } 
      //left joystick left
      if (value == "left") {
        digitalWrite(IN12, LOW);
        digitalWrite(IN14, LOW);
        digitalWrite(IN27, HIGH);
        digitalWrite(IN26, LOW);
        response = "turning left";
      }
      //left joystick right
      if (value == "right") {
        digitalWrite(IN12, HIGH);
        digitalWrite(IN14, LOW);
        digitalWrite(IN27, LOW);
        digitalWrite(IN26, LOW);
        response = "turning right";
      }
      //left joystick down
      if (value == "reverse") {
        digitalWrite(IN12, LOW);
        digitalWrite(IN14, HIGH);
        digitalWrite(IN27, LOW);
        digitalWrite(IN26, HIGH);
        response = "reversing";
      }
      //left joystick center
      if (value == "park") {
        digitalWrite(IN12, LOW);
        digitalWrite(IN14, LOW);
        digitalWrite(IN27, LOW);
        digitalWrite(IN26, LOW);
        response = "in park";
      }
      //right joystick up
      //tilt servo up
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
      //pan servo left
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
      //pan servo right
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
      //tilt servo down
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
      } 
      //L2 button
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
      webSocket.sendTXT(num, response);         
    }
  }
}
