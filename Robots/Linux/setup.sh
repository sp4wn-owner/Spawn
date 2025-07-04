#!/bin/bash

# Exit on any error
set -e

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install Curl Node (setup Node.js LTS)
echo "Setting up Node.js LTS..."
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -

# Install Node.js
echo "Installing Node.js..."
sudo apt-get install -y nodejs

# Install node-pre-gyp globally
echo "Installing node-pre-gyp..."
sudo npm install -g @mapbox/node-pre-gyp

# Create bot directory and navigate into it
echo "Creating bot directory..."
mkdir -p bot
cd bot

# Install required Node.js modules
echo "Installing Node.js modules (node, ws, wrtc)..."
npm install node ws wrtc

# Download scripts from GitHub
echo "Downloading client.js, vrHandler.py, config.js, and camera.py..."
wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/blob/main/Robots/Linux/client.js
wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/blob/main/Robots/Linux/vrHandler.py
wget https://raw.githubusercontent.com/sp4wn-owner/Spawn/blob/main/Robots/Linux/config.js

echo "Setup complete!"