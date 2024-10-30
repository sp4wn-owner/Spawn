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

## Connect your Raspberry Pi robot:

### To SSH into your Pi you'll need PuTTY and FileZilla or similar programs

### To set up SSD card you'll need to install the Raspberry Pi Imager
```bash
sudo apt update
sudo apt full-upgrade
sudo raspi-config

### then update

sudo rpi-update

sudo nano /boot/firmware/config.txt

### default is 1, but you'll need to change this to 0
camera_auto_detect=0

### add the following three lines at the bottom of this file
start_x=1
gpu_mem=128
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

### Reboot your Pi
sudo reboot

### Install Node.js and FFmpeg
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install ffmpeg

### Install Node.js if you plan to stream to twitch
sudo apt-get install -y nodejs
sudo npm install -g node-pre-gyp --force

mkdir bot
cd bot
npm install node ws sharp wrtc @sp4wn/pipins

sudo chmod a+rw /dev/video0
sudo node piclient.js # run your script after transferring the file over using FileZilla

### Starting script at boot and restarting after cleanup
sudo nano /etc/systemd/system/piclient.service

### Add the following text into this file and save
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

### Reload the systemd daemon and enable the service
sudo systemctl daemon-reload
sudo systemctl enable piclient.service

### Check the service status
sudo systemctl status piclient.service

### Stop the piclient service
sudo systemctl stop piclient.service

### Inspect logs from piclient
journalctl -u piclient.service







