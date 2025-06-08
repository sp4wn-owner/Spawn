# Teleoperation

## Step 1
Follow the installation instructions. Communication with arms is over the can0 and can1. Please refer to AgileX's [GitHub](https://github.com/agilexrobotics/piper_sdk/blob/master/README(EN).MD) for setting up communication. 

Our vrHandler script is currently untested with the hardware. You'll most likely need to update the IK to be compatible with your setup. This is the JSON output being received by the vrHandler: 

```bash
{
  "head": {
    "position": {
      "x": 0.0,
      "y": 1.6,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  },
  "controllers": [
    {
      "gamepad": {
        "hand": "left",
        "buttons": [
          { "pressed": false, "value": 0.0 },
          { "pressed": true, "value": 1.0 }
        ],
        "axes": [0.0, 0.0]
      },
      "gripPosition": {
        "x": -0.4,
        "y": 1.2,
        "z": 0.6
      },
      "gripOrientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    },
    {
      "gamepad": {
        "hand": "right",
        "buttons": [
          { "pressed": false, "value": 0.0 },
          { "pressed": false, "value": 0.0 }
        ],
        "axes": [0.0, 0.0]
      },
      "gripPosition": {
        "x": 0.4,
        "y": 1.2,
        "z": 0.6
      },
      "gripOrientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  ]
}
```

## Step 2
On [Spawn](https://sp4wn.com), connect to your robot (sp4wn.com/username). Then 'Enter VR' to start transmitting tracking data. 

## Installation
```bash
sudo sh -c "wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/Robots/AgileX/Piper_Arms/setup.sh && chmod +x setup.sh && ./setup.sh"
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
