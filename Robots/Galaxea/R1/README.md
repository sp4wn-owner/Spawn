# Galaxea R1 - Teleoperation

## Step 1
Follow the installation instructions.

## Step 2
On [Spawn](https://sp4wn.com), connect to your robot (sp4wn.com/username). Then 'Enter VR' to start transmitting tracking data. 

## Installation
```bash
sudo sh -c "wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/main/Robots/Galaxea/R1/setup.sh && chmod +x setup.sh && ./setup.sh"
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
# Isaac Sim

## Step 1
After spawning your Galaxea model in the Isaac Sim environment, deploy the simHandler script and start the server.

## Step 2
In the Robot-UI, add your websocket URL in the input field and then start streaming.

## Step 3
From the VR-UI, connect to your robot by clicking the 'spawn' button. Tracking data will be proxied from the Robot-UI to the simHandler script thus interfacing with ROS to manipulate the Galaxea R1 model in a simulated environment.
