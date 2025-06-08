#!/bin/bash

# Curl Node
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -

# Install Node.js
sudo apt-get install -y nodejs

# WebRTC requires node-pre-gyp
sudo npm install -g @mapbox/node-pre-gyp

# Make bot directory for your script
mkdir bot
cd bot

# Install node modules in bot directory
npm install node ws wrtc @sp4wn/pipins

# Enable camera access
sudo chmod a+rw /dev/video0

# Get the piclient, vrHandler, and config scripts
wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/blob/main/Robots/Pi/Raspberry%20Pi%20Zero%202W/client.js https://raw.githubusercontent.com/sp4wn-owner/Spawn/blob/main/Robots/Pi/Raspberry%20Pi%20Zero%202W/vrHandler.js https://raw.githubusercontent.com/sp4wn-owner/Spawn/blob/main/Robots/Pi/Raspberry%20Pi%20Zero%202W/config.js
