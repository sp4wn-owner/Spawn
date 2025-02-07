## Testing Setup
After starting neck_server.py, connect to your ws using the [Robot-UI](https://robot-csyy.onrender.com) by entering the URL and clicking start. Then on the [VR-UI](https://spawn-vr.onrender.com) enter your robot's username and click "Spawn". After connecting, click "Enter VR". Now you should see all head tracking data [w,x,y,z] being proxied to your neck_server. 

### Example data from Robot-UI to neck_server.py
![Alt text](https://github.com/sp4wn-owner/Spawn/blob/main/Spawn-VR/Dropbear/Images/wsdata.png)

From the neck_server script, uncomment the block to forward this data to serial_handler.py. From there, it forwards the data to your ESP32.

Please note, the move_head script hasn't been tested yet so please review the calculations and make necessary adjustments to avoid damaging components. This script is merely a starting point and will probably require modifications. The objective is to minimize latency for real-time teleoperation of head movements with 6DoF. My proposed solution is to implement a learning-based approach paired with gradient descent. Over time, this should result in lower latency compared to existing time-consuming IK calculations.

## Teleoperation:

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
npm install node ws wrtc
```

### Enable camera access
```bash
sudo chmod a+rw /dev/video0
```

### Get the client, serial_handler, and config scripts
```bash
wget https://github.com/sp4wn-owner/Spawn/tree/main/Spawn-VR/Dropbear/client.js https://github.com/sp4wn-owner/Spawn/tree/main/Spawn-VR/Dropbear/serial_handler.py https://github.com/sp4wn-owner/Spawn/tree/main/Spawn-VR/Dropbear/config.js
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

## Helpful commands

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