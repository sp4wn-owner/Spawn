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
