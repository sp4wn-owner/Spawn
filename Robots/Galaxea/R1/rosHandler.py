# This script serves as the middleware between our client and the ROS. 
# This is unique to your robot and the ROS you are using. 
# Generally, manufacturers are responsible for implementing this logic.
# This script listens for messages from the client, sends them to the ROS, and returns the response back to the client. 
# The ROS can be any system that handles and converts the tracking data into actionable commands.