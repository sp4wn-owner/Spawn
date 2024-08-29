# microcontrollers

ESP32:
v0_robot:
Simply flash the esp32 using the arduino IDE at 115200 baud rate then you should be able to connect it to your web browser on sp4wn.com either before or after going live. I used an l293d to control the motors. Video tutorial coming soon.

CameraWebServer & ESP Web Server: 
The CameraWebServer turns your ESP32 into an IP camera. The ESP Web Server is a socket server that allows you to control your robot. To use these on the website you'll need to run the web page locally or convert the CameraWebServer into an HTTPS server and the socket server into a WSS. If you're able to get it working as an HTTPS/WSS server please let me know. I haven't looked into it much yet.



