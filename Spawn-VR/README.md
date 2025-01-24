# Spawn-VR

Spawn-VR is an open-source VR web client tailored for telepresence robotics applications. It empowers developers to create custom interfaces and offers the option to connect to Spawn for quick deployment and testing via secure peer-to-peer connections.

## Features: 

- Built-in cryptocurrency (deposit/withdraw on Spawn)  
    &nbsp;&nbsp;&nbsp;&nbsp;- Connecting to Spawn gives you access to the built-in cryptocurrency, $SPAWN token, making it easy to incentivize or monetize controlling your robot.

- Real-time control  
    &nbsp;&nbsp;&nbsp;&nbsp;- Experience ultra-low latency enabling near real-time control over the internet. Perfect for teleoperation/telepresence applications.

-  Network agnostic  
    &nbsp;&nbsp;&nbsp;&nbsp;- Operates seamlessly over any network and designed with Starlink connectivity in mind. Devices connected via satellite can be anywhere in the world.

- Secure  
    &nbsp;&nbsp;&nbsp;&nbsp;- Optionally handle your own security locally thus eliminating any potential access to your device(s) in the event of a security breach on Spawn or your own network. Passwords/codes are securely hashed and all data is encrypted. No personal information is requested nor retained. Update the security parameters in the robot's client.js file.

## Steps:

### Create accounts
- On [Spawn](https://sp4wn.com), create two accounts: one for the robot and one for you. 

### Update client.js
- For both the Robot UI and VR UI, update the account information with your username/password

### Open index.html
- Using a web browser, open the index.html file for both the Robot UI and VR UI. This can be done locally or hosted. 

### Start the robot's stream
- Click the "Start" button to make your robot simulator accessible. 

### Connect to your robot from the VR-UI client
- Click the "Spawn" button to create a connection to your robot.

### Update your accounts on Spawn

## TODO
- Integrate the VR system into the VR UI