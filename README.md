[![Features](https://img.shields.io/badge/-Features-blue?style=for-the-badge)](#Features)
[![Pi Robot](https://img.shields.io/badge/-Pi%20Robot-purple?style=for-the-badge)](#Pi-Robot)
[![Phone Robot](https://img.shields.io/badge/-Phone%20Robot-green?style=for-the-badge)](#Phone-Robot)

# [Spawn](https://sp4wn.com)

Spawn is telepresence software for advanced robots. Its quick setup time facilitates the rapid deployment of custom telepresence robots and devices without the complexities of networking or communication management. 

### Before setting up your robot, create an account on [Spawn](https://sp4wn.com)
- Accounts are anonymous, yet required to access the application until the guest layer is implemented. Streams are securely sent through a direct p2p connection thus keeping sessions private and resulting in the lowest latency possible. However, in the event that a p2p connection could not be properly established, traffic will be routed through a global network of servers leading to a slight increase in latency. 

## Key Features:

- Free to use  
    &nbsp;&nbsp;&nbsp;&nbsp;- Because most connections are direct peer-to-peer, we avoid the need for an expensive network of servers to route data and process or store media. Should we see an overwhelming number of connections requiring data routing then we may implement a paid option for this service.

- Real-time control  
    &nbsp;&nbsp;&nbsp;&nbsp;- Experience ultra-low latency enabling near real-time control over the internet. 

-  Network agnostic  
    &nbsp;&nbsp;&nbsp;&nbsp;- Operates seamlessly over any network and designed with Starlink connectivity in mind. Devices connected via satellite can be anywhere in the world.

- Secure  
    &nbsp;&nbsp;&nbsp;&nbsp;- Optionally handle your own security locally thus eliminating any potential access to your device(s) in the event of a security breach on Spawn or your own network. Passwords/codes are securely hashed and all data is encrypted. No personal information is requested nor retained. Update the security parameters in the config.js file on your Raspberry Pi or other client implementation.

- Monetization  
    &nbsp;&nbsp;&nbsp;&nbsp;- Transform your telepresence robots/devices into revenue-generating assets or pay users for controlling them (i.e.remote telepresence gig). We take 20% of token redemptions to maintain and grow the platform. Setting your token rate to a negative number will deduct tokens per min from your account to the other user while a positive token rate will deduct tokens from the user to your account. Setting the token rate to zero will enable free access.

- Public/private access  
    &nbsp;&nbsp;&nbsp;&nbsp;- Share your robot with the world, add a secret code for authenticated access, and toggle visibility in our live feed. Robots removed from the live feed can still be accessed at sp4wn.com/[username]. In this case, your username could double as a private key. Following the robot (clicking the star icon) will include it in your feed regardless of visibility, enabling a customizable feed of both public and personal robots/devices.

- Quick and easy setup (no coding initially required)  
    &nbsp;&nbsp;&nbsp;&nbsp;- We provide all the code necessary to connect your bot to our platform and start teleoperating in minutes. You have full control over how to handle the received input commands. Controllers/gamepads are currently supported. VR support is in development.

## Pi Robot:

### To SSH into your Pi you'll need [PuTTY](https://www.putty.org/) and [FileZilla](https://filezilla-project.org/download.php?platform=win64) or similar programs  

### To set up the SSD card you'll need to install the [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

### Hardware
- Raspberry Pi (I'm using the Zero 2W, but other models should work as well)
- Robot chassis (You can find a cheap robot chassis and other parts on Aliexpress/Amazon/eBay or 3D print your own)
- RPI camera
- Camera mount
- SD card
- L293D (2wd)/L293N (4wd) or another motor driver
- Breadboard
- Battery or power supply with respective connectors (I'm using a USB breakout to power the motors from the battery)
- Breadboard jumper wires 
- Soldering iron (If you'd like to fuse the wires to the motor connectors so they don't disconnect)

### From a clean install of Raspberry Pi OS (I'm using the headless lite version), update your Pi
```bash
sudo apt update -y && sudo apt full-upgrade -y
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
CTRL+X then Y then Enter to save

### Reboot your Pi
```bash
sudo reboot
```

### Curl Node
```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
```

### Install Node.js
```bash
sudo apt-get install -y nodejs
```

### WebRTC requires node-pre-gyp
```bash
sudo npm install -g @mapbox/node-pre-gyp
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

### Get the piclient, inputHandler, and config scripts
```bash
wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/main/Pi/Raspberry%20Pi%20Zero%202W/piclient.js https://raw.githubusercontent.com/sp4wn-owner/Spawn/main/Pi/Raspberry%20Pi%20Zero%202W/inputHandler.js https://raw.githubusercontent.com/sp4wn-owner/Spawn/main/Pi/Raspberry%20Pi%20Zero%202W/config.js
```

### Update the config.js file with your username/password
```bash
nano config.js
```
CTRL+X then Y then Enter to save

### (Optional) Update the inputHandler.js file with your custom control commands
```bash
nano inputHandler.js
```
CTRL+X then Y then Enter to save

### Run the script
This script will automatically connect to https://sp4wn.com. Because all connections are p2p, you will be required to create a different account to connect to your robot. Only one session per account is allowed at a time.
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

### Helpful command to see a list of devices (if you don't see video0, make sure your camera is properly connected)
```bash
ls /dev/ | grep video
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
- Flash ESP32 using the Arduino IDE with respective code (ESP32_BLE_Dev_Module) from our [github](https://github.com/sp4wn-owner/Spawn/blob/main/ESP32/ESP32_BLE_Dev_Module/ESP32_BLE_Dev_Module.ino)
- If this is your first time using the ESP32 with the Arduino IDE, you'll need to add https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json to additional boards manager (file > preferences)
- To flash this board you may need to update your USB driver (see file). Then select board (Tools > Board > esp32 > ESP Dev Module).

### Step 2
- Connect all wires and secure phone mount
- On the Spawn website under profile, click "Go Live" then click "Connect". Your robot should pop up on your list of Bluetooth devices. Once connected, test using the controls.

### Other
- The esp32 can be powered through the micro USB or through the 5v pin (voltage above 3.3 will be stepped down through this pin since the board operates at 3.3v). If the motors are connected to the same power source they may cause the board to reset if not properly regulated.
- Edit the code and make your robot do whatever you want when it receives certain messages.
- If Bluetooth is not enabled you will need to go to chrome://flags to enable it



