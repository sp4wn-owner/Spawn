# Spawn-VR

Spawn-VR is an open-source VR framework tailored for telepresence robotics applications. It empowers developers to create custom interfaces and offers the option to connect to Spawn for quick deployment and testing via secure peer-to-peer connections. VR Test portal available [here](https://spawn-vr.onrender.com)

## Features: 

- Integrated cryptocurrency (deposit/withdraw on Spawn)  
    &nbsp;&nbsp;&nbsp;&nbsp;- Connecting your robot to Spawn provides access to our $SPAWN token, facilitating seamless incentivization or monetization of your robot.

- Real-time control  
    &nbsp;&nbsp;&nbsp;&nbsp;- Experience ultra-low latency enabling near real-time control over the internet. Perfect for teleoperation/telepresence applications.

-  Network agnostic  
    &nbsp;&nbsp;&nbsp;&nbsp;- Operates seamlessly over any network and designed with Starlink connectivity in mind. Devices connected via satellite can be anywhere in the world.

- Secure  
    &nbsp;&nbsp;&nbsp;&nbsp;- Optionally handle your own security locally thus eliminating any potential access to your device(s) in the event of a security breach on Spawn or your own network. Passwords/codes are securely hashed and all data is encrypted. No personal information is requested nor retained. Update the security parameters in the robot's client.js file or on Spawn.

## Steps:

### Create accounts
- On [Spawn](https://sp4wn.com), create at least one account for your robot. 

### Update Robot-UI
- Edit the client.js file to include your username and password. The VR-UI will automatically log in as a guest.

### Open index.html files
- Open the index.html file for the Robot UI in a web browser. While the Robot UI can be run locally, the VR UI requires HTTPS. You can test the VR UI using the [portal](https://spawn-vr.onrender.com)

### Start the robot's stream
- Click the "Start" button to connect your robot to Spawn. 

### From the VR-UI connect to your robot
- Enter your robot's username and click the "Spawn" button to connect to your robot.

### Update your accounts on Spawn
- Deposit/withdraw tokens
