# [Spawn](https://sp4wn.com)

Spawn is at the forefront of the humanoid gig economy, developing the essential infrastructure to enable real-time teleoperation and monetization of advanced robots.

### Supported systems
Our software is compatible with leading Linux distributions.

### Getting started
Please refer to the Robots directory for setup instructions and Spawn-VR for teleoperating your robot using VR. 

## Key features
- Real-time control  
    &nbsp;&nbsp;&nbsp;&nbsp;- Experience ultra-low latency enabling near real-time control over the internet. 

-  Network agnostic  
    &nbsp;&nbsp;&nbsp;&nbsp;- Operates seamlessly over any network and designed with Starlink connectivity in mind. Robots connected via satellite can be nearly anywhere in the world.

- Secure  
    &nbsp;&nbsp;&nbsp;&nbsp;- Optionally handle your own security. Passwords/codes are securely hashed and all data is encrypted. No personal information is requested nor retained. Update the security parameters in the config.js file.

- Token system (cryptocurrency)  
    &nbsp;&nbsp;&nbsp;&nbsp;- Transform your telepresence robots into revenue-generating assets or pay users for controlling them (i.e.remote telepresence gig). We take 20% of token redemptions to maintain and grow the platform. Setting your robot's token rate to a negative number will deduct tokens from your account to the user while a positive token rate will deduct tokens from the user to your account. Setting the token rate to zero will enable free access. Our system functions similar to a centralized exchange with automatic deposits and withdrawals.

- Public/private access  
    &nbsp;&nbsp;&nbsp;&nbsp;- Share your robot with the world, add a secret code for authenticated access, and toggle visibility in our live feed. Robots removed from the live feed can still be accessed at sp4wn.com/[username]. Following the robot (clicking the star icon) will include it in your feed regardless of visibility, enabling a customizable feed of both public and personal robots.

- Easy integration  
    &nbsp;&nbsp;&nbsp;&nbsp;- We provide all the code necessary to connect your robot to our platform. However, you will be required to build the middleware that pipes the tracking data into ROS or any system you're using. Please refer to our examples or reach out to us for assistance.