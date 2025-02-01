# Spawn-VR

Spawn-VR is an open-source VR framework tailored for telepresence robotics applications. It empowers developers to create custom interfaces and offers the option to connect to Spawn for quick deployment and testing via secure peer-to-peer connections.
<div align="center">
  <a href="https://youtu.be/RPz4-uel9Uk?si=XMEG3FItI1AMAv_V" target="_blank">
    <img src="https://img.youtube.com/vi/RPz4-uel9Uk/0.jpg" alt="Watch the video">
  </a>
</div>

## Instructions:

The platform is functional. Test it out by opening two browser tabs: one for the [VR-UI](https://spawn-vr.onrender.com) and one for the [Robot-UI](https://robot-csyy.onrender.com). Just install the WebXR Chrome extension and open the developer tools (same category as elements and console) to simulate the 3D immersive environment.

### Create accounts
- On [Spawn](https://sp4wn.com), create at least one account for your robot. 

### Robot login
- Enter your username and password, and then click the "Start" button to connect to Spawn. The Robot-UI can be run locally or through our [portal](https://robot-csyy.onrender.com). Open the developer tools to inspect incoming tracking data. Connect to your simulation server to forward tracking data. See "simserver.py" for a demo script. 

### VR login
- You'll be automatically logged in as a guest, but you're welcome to log in with your username and password. Just enter your robot's username and then click the "Spawn" button to connect to your robot. The VR-UI requires HTTPS, so you'll either need to install a certificate to run it locally or use a hosting provider. You may also use our [portal](https://spawn-vr.onrender.com). 

### VR tips
- In the Oculus browser, navigate to your VR-UI or use our portal [spawn-vr.onrender.com].
- This application isn't optimized yet so you may need to re-enter VR mode if you see a black screen. 

### Update your accounts on Spawn
- Deposit/withdraw tokens
- Toggle secret code
- Toggle public visibility
- Update account details

## Features: 

- Integrated cryptocurrency (deposit/withdraw on Spawn)  
    &nbsp;&nbsp;&nbsp;&nbsp;- Connecting your robot to Spawn provides access to our $SPAWN token, facilitating seamless incentivization or monetization of your robot.

- Real-time control  
    &nbsp;&nbsp;&nbsp;&nbsp;- Experience ultra-low latency enabling near real-time control over the internet. Perfect for teleoperation/telepresence applications.

-  Network agnostic  
    &nbsp;&nbsp;&nbsp;&nbsp;- Operates seamlessly over any network and designed with Starlink connectivity in mind. Devices connected via satellite can be anywhere in the world.

- Secure  
    &nbsp;&nbsp;&nbsp;&nbsp;- Optionally handle your own security locally thus eliminating any potential access to your device(s) in the event of a security breach on Spawn or your own network. Passwords/codes are securely hashed and all data is encrypted. No personal information is requested nor retained. Update the security parameters in the robot's client.js file or on Spawn.
