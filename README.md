# [Spawn](https://sp4wn.com)

Spawn is an AI/teleoperation platform for advanced robots. We handle all the networking, security, and authentication so you can quickly start teleoperating your robot or device within minutes. Best of all, it's free for non-commercial use. 

### Getting started
Please refer to the Robots directory for setup instructions.

## Key features
- Real-time control  
    &nbsp;&nbsp;&nbsp;&nbsp;- Experience ultra-low latency enabling near real-time control over the internet via direct peer-to-peer connections. 

- Secure  
    &nbsp;&nbsp;&nbsp;&nbsp;- Optionally handle your robot's security. Passwords/codes are securely hashed and all data is encrypted. No personal information is requested nor retained. Update the security parameters in the config.js file.

- Token system (cryptocurrency)  
    &nbsp;&nbsp;&nbsp;&nbsp;- Transform your robots into revenue-generating assets or pay users for controlling them. Initially, we intend to burn 20% of token redemptions. Setting your robot's token rate to a negative number will deduct tokens from your account to the user while a positive token rate will deduct tokens from the user to your account. Setting the token rate to zero will enable free access. Our system functions similar to a centralized exchange with automatic deposits and withdrawals.

- Public/private access  
    &nbsp;&nbsp;&nbsp;&nbsp;- Share your robot with the world, add a secret code for authenticated access, and toggle visibility in our live feed. Robots removed from the live feed can still be accessed at sp4wn.com/[username]. Following the robot (clicking the star icon) will include it in your feed regardless of visibility, enabling a customizable feed of both public and personal robots.

- Easy integration  
    &nbsp;&nbsp;&nbsp;&nbsp;- We provide all the code necessary to connect your robot to our platform. However, you will be required to build/update the middleware (vrHandler) that controls the robot. Please refer to our examples or reach out to us for assistance.