# Unitree G1 EDU

## Step 1
Follow the installation instructions.

## Step 2
On [Spawn](https://sp4wn.com), connect to your robot (sp4wn.com/username). Then 'enter immersive' to start transmitting tracking data. 

## Installation - Not ready

### Curl Node
```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
```

### Install Node.js
```bash
sudo apt-get install -y nodejs
```

### Install node-pre-gyp
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
npm install node ws wrtc
```

### Enable camera access
```bash
sudo chmod a+rw /dev/video0
```

### Get the client, serial_handler, and config scripts
```bash
wget https://github.com/sp4wn-owner/Spawn/tree/main/Robots/Unitree/G1/client.js https://github.com/sp4wn-owner/Spawn/tree/main/Unitree/G1/Dropbear/serial_handler.py https://github.com/sp4wn-owner/Spawn/tree/main/Unitree/G1/Dropbear/config.js
```

### Update the config.js file with your username/password
```bash
nano config.js
```
CTRL+X then Y then Enter to save

### Run the script
This script connects your robot to Spawn.
```bash
sudo node client.js
```
## Optional

### Starting script at boot and restarting after cleanup
```bash
sudo nano /etc/systemd/system/client.service
```

### Add the following text into this file and save
```bash
[Unit]
Description=Start client.js script
After=network.target

[Service]
ExecStart=/usr/bin/node /home/nvidia/bot/client.js
WorkingDirectory=/home/nvidia/bot
User=nvidia  # Specify the user to run the service as
Group=nvidia # Optional, if group is different from user
Restart=always
RestartSec=10
StandardOutput=append:/home/nvidia/bot/client.log
StandardError=append:/home/nvidia/bot/client.log

[Install]
WantedBy=multi-user.target
```

### Reload the systemd daemon
```bash
sudo systemctl daemon-reload
```

### Enable the service
```bash
sudo systemctl enable client.service
```

### Immediately start the service (if enabled, service will start on boot)
```bash
sudo systemctl start client.service
```

### Disable the service
```bash
sudo systemctl disable client.service
```

### Check the service status
```bash
sudo systemctl status client.service
```

### Stop the service
```bash
sudo systemctl stop client.service
```

### Restart the service
```bash
sudo systemctl restart client.service
```

### Inspect service logs
```bash
sudo journalctl -u client.service -f
```

### List of devices (if you don't see video0, make sure your camera is properly connected)
```bash
ls /dev/ | grep video
```
