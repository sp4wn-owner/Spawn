[![Features](https://img.shields.io/badge/-Features-blue?style=for-the-badge)](#features)
[![Pi Robot](https://img.shields.io/badge/-Pi%20Robot-purple?style=for-the-badge)](#Pi-Robot)

# [Spawn](https://sp4wn.com)

A platform where you can share access to your telepresence robot without worrying about networking/communication protocols. Just build your bot, flash our code onto your microcontroller, and you're ready to go! 

**Please note that WebRTC may not be functioning properly on iOS.**

- Currently supporting the following robot types: phone + ESP32 || Raspberry Pi


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

### To SSH into your Pi you'll need PuTTY and FileZilla or similar programs

### To set up SSD card you'll need to install the Raspberry Pi Imager
```bash
sudo apt update
sudo apt full-upgrade
```
### Update raspi-config
```bash
sudo raspi-config
```

### Fully update Pi
```bash
sudo rpi-update
```

### Open config.txt and setup camera/pwm channels
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

### Install Node.js and FFmpeg
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
npm install node ws sharp wrtc @sp4wn/pipins
```

### Enable camera access
```bash
sudo chmod a+rw /dev/video0
```

### Transfer script from our Raspberry Pi directory to your bot directory then run it
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







