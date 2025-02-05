## After starting neck_server.py, connect to your ws using the Robot-UI by entering the URL and clicking start. Then on the VR-UI enter your robot's username and click "Spawn". After connecting, click "Enter VR". Now you should see all head tracking data [w,x,y,z] being proxied to your neck_server. 

## From the neck_server script, uncomment the block to forward this data to serial_handler.py. From there, it forwards the data to your ESP32.

## The move_head.cpp handles the gradient descent calculations enabling simplified 6dof neck movements. Please note, this has not been tested yet as I do not have the motors so please review the script and calculations and make necessary adjustments. This script is merely a starting point and will probably require modifications.