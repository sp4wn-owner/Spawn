[![Features](https://img.shields.io/badge/-Features-blue?style=for-the-badge)](#Features)
[![Pi Robot](https://img.shields.io/badge/-Pi%20Robot-purple?style=for-the-badge)](#Pi-Robot)
[![Phone Robot](https://img.shields.io/badge/-Phone%20Robot-purple?style=for-the-badge)](#Phone-Robot)

# [Spawn](https://sp4wn.com)

A platform where you can share access to your telepresence robot without worrying about networking/communication protocols. Just build your bot, copy our code, modify controls as you please, and you're ready to go! 

**Please note that WebRTC may not be functioning properly yet on iOS.**

- Currently supporting the following robot types: Raspberry Pi || Phone + ESP32.

## Features:

- Quick and easy setup (no coding required)  
    &nbsp;&nbsp;&nbsp;&nbsp;- We provide all the code necessary to get your bot up and running.

- Near real-time sub-second latency  
    &nbsp;&nbsp;&nbsp;&nbsp;- Teleoperate robots with minimal delay. 

- Charge for access to your robot (tokens/min)  
    &nbsp;&nbsp;&nbsp;&nbsp;- i.e. Peer A sets the number of tokens per minute for their robot. Peer B is charged the rate/min every minute until they disconnect. Turn telepresence robots into income-generating assets.

- Modifiable code to handle input controls (gamepads/controllers are supported)  
    &nbsp;&nbsp;&nbsp;&nbsp;- i.e. Peer A shares access to their robot. Peer B controls the robot with a controller. Peer A's robot can interpret the "A" button press however they'd like (move servo to position, execute function, etc.)

## Pi Robot:

### To SSH into your Pi you'll need [PuTTY](https://www.putty.org/) and [FileZilla](https://filezilla-project.org/download.php?platform=win64) or similar programs  

### To set up the SSD card you'll need to install the [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

### Hardware
- Raspberry Pi (I'm using the Zero 2W, but other models should work as well)
- Robot chassis (You can find a cheap robot chassis on Aliexpress/Amazon/eBay or 3D print your own)
- RPI camera
- SD card
- L293D or another motor driver
- Breadboard
- Battery or power supply with respective connectors (I'm using a USB breakout to power the motors from the battery)
- Breadboard jumper wires 
- Soldering iron (If you'd like to fuse the wires to the motor connectors so they don't disconnect)

### From a clean install of Debian (I'm using the headless version), update your Pi
```bash
sudo apt update
sudo apt full-upgrade
```

### Open config.txt and set up camera/pwm channels
```bash
sudo nano /boot/firmware/config.txt
```

### Since we're using v4l2 you'll need to change this to 0
```bash
camera_auto_detect=0
```

### Add the following three lines at the bottom of the config.txt file
```bash
start_x=1
gpu_mem=128
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

### Reboot your Pi
```bash
sudo reboot
```

### Curl Node
```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
```

### Install ffmpeg if you'd like to stream your robot to Twitch
```bash
sudo apt-get install ffmpeg
```

### Install Node.js
```bash
sudo apt-get install -y nodejs
```

### WebRTC requires node-pre-gyp
```bash
sudo npm install -g node-pre-gyp --force
```

### Make bot directory for your script
```bash
mkdir bot
cd bot
```

### Install node modules in bot directory
```bash
npm install node ws wrtc @sp4wn/pipins
```

### Enable camera access
```bash
sudo chmod a+rw /dev/video0
```

### Get the piclient script
```bash
wget https://raw.githubusercontent.com/sp4wn-owner/microcontrollers/main/RPI/Raspberry%20Pi%20Zero%202W/piclient.js
```

### Update the file and add your username/password plus any additional changes to control commands from the input channel.
```bash
nano piclient.js
```
CTRL+X then Y then Enter to save

### Run the script
This script will automatically connect to https://sp4wn.com. Because all connections are peer-to-peer you will be required to create a different account to connect to your robot. Only one session per account is allowed at a time.
```bash
sudo node piclient.js
```

### Starting script at boot and restarting after cleanup
```bash
sudo nano /etc/systemd/system/piclient.service
```

### Add the following text into this file and save
```bash
[Unit]
Description=Start piclient.js script
After=network.target

[Service]
ExecStart=/usr/bin/node /home/pi/bot/piclient.js
WorkingDirectory=/home/pi/bot
Restart=always
RestartSec=10
StandardOutput=append:/home/pi/bot/piclient.log
StandardError=append:/home/pi/bot/piclient.log

[Install]
WantedBy=multi-user.target
```

### Reload the systemd daemon and enable the service
```bash
sudo systemctl daemon-reload
```
```bash
sudo systemctl enable piclient.service
```

### Check the service status
```bash
sudo systemctl status piclient.service
```

### Stop the piclient service if you'd like
```bash
sudo systemctl stop piclient.service
```

### Inspect logs from piclient
```bash
journalctl -u piclient.service
```

## Phone Robot:

### Hardware (Phone + ESP32)
- Mobile phone
- ESP32 WROOM 32D microcontroller
- Robot chassis (You can find a cheap robot chassis on Aliexpress/Amazon/eBay or 3D print your own)
- L293D or another motor driver
- Breadboard
- Phone mount (1/4-20 x 3/4" Truss Head Machine Screws + 1/4-20 Inch Wingnuts) [ebay](https://www.ebay.com/itm/335118194262)
- Battery or power supply with respective connectors (I'm using a USB breakout to power the motors from the battery)
- Breadboard jumper wires 
- Soldering iron (If you'd like to fuse the wires to the motor connectors so they don't disconnect)

### Step 1
- Flash ESP32 using the Arduino IDE with respective code (ESP32_BLE_Dev_Module) from our [github](https://github.com/sp4wn-owner/microcontrollers/blob/main/Body/ESP32_BLE_Dev_Module/ESP32_BLE_Dev_Module.ino)
- If this is your first time using the ESP32 with the Arduino IDE, you'll need to add https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json to additional boards manager (file > preferences)
- To flash this board you may need to update your USB driver (see file). Then select board (Tools > Board > esp32 > ESP Dev Module).

### Step 2
- Connect all wires and secure phone mount
- On the Spawn website under profile, click "Go Live" then Click "Connect". Your robot should pop up on your list of Bluetooth devices. Once connected, test using the controls.

### Other
- The esp32 can be powered through the micro USB or through the 5v pin (voltage above 3.3 will be stepped down through this pin since the board operates at 3.3v). If the motors are connected to the same power source they may cause the board to reset if not properly regulated.
- Edit the code and make your robot do whatever you want when it receives certain messages.
- If Bluetooth is not enabled you will need to go to chrome://flags to enable it



