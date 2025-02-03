# Teleoperating Galaxea R1

## Step 1
Deploy the client and rosHandler scripts (to be completed).

## Step 2
Run the client script ensuring connection to Spawn.

## Step 3
From the VR-UI, connect to your Galaxea robot. Then 'enter immersive' to start transmitting tracking data. 

# Isaac Sim

## Step 1
After spawning your Galaxea model in the Isaac Sim environment, deploy the simHandler script and start the server.

## Step 2
In the Robot-UI, add your websocket URL in the input field and then start streaming.

## Step 3
From the VR-UI, connect to your robot by clicking the 'spawn' button. Tracking data will be proxied from the Robot-UI to the simHandler script thus interfacing with ROS to manipulate the Galaxea in a simulated environment.

