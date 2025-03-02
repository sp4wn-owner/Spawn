import multiprocessing
import traceback
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Bool, String
import time

class RobotControlNode:
    def __init__(self, pipe):
        # Store the pipe for communication
        self.pipe = pipe

        # Publishers from JointStateSender
        self.left_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_arm_left', JointState, queue_size=10)
        self.right_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_arm_right', JointState, queue_size=10)
        self.torso_joint_state_pub = rospy.Publisher(
            '/torso_joint_target_position_', JointState, queue_size=10)
        self.torso_joint_state_pub_real = rospy.Publisher(
            '/motion_target/target_joint_state_torso', JointState, queue_size=10)
        self.chassis_command_pub = rospy.Publisher(
            '/motion_target/target_speed_chassis', Twist, queue_size=10)
        self.acc_limit_pub = rospy.Publisher(
            '/motion_target/chassis_acc_limit', Twist, queue_size=10)
        self.braking_mode_pub = rospy.Publisher(
            '/motion_target/brake_mode', Bool, queue_size=10)

        # Subscriber for VR headset quaternion (head orientation)
        rospy.Subscriber('vr_head_cmd', Quaternion, self.head_command_callback)

        # Response publisher
        self.response_pub = rospy.Publisher('robot_response', String, queue_size=10)

        # Placeholder for controller data
        self.controller_data = {'left': None, 'right': None}

        rospy.loginfo('Robot Control Node initialized')

    def send_joint_state(self, position_left, position_right, position_torso):
        """Publish joint states for left arm, right arm, and torso."""
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub.publish(torso_joint_state)

    def send_joint_state_real(self, position_left, position_right, position_torso):
        """Publish real joint states for left arm, right arm, and torso."""
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)

    def send_vel_limit(self, vel_limit_left, vel_limit_right, vel_limit_torso):
        """Publish velocity limits for left arm, right arm, and torso."""
        left_joint_state = JointState()
        left_joint_state.velocity = vel_limit_left
        right_joint_state = JointState()
        right_joint_state.velocity = vel_limit_right
        torso_joint_state = JointState()
        torso_joint_state.velocity = vel_limit_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)

    def send_chassis(self, chassis_command):
        """Publish chassis speed command."""
        chassis_command_msg = Twist()
        chassis_command_msg.linear.x = chassis_command[0]
        chassis_command_msg.linear.y = chassis_command[1]
        chassis_command_msg.angular.z = chassis_command[2]
        self.chassis_command_pub.publish(chassis_command_msg)

    def send_acc_limit(self, acc_limit):
        """Publish chassis acceleration limit."""
        acc_limit_msg = Twist()
        acc_limit_msg.linear.x = acc_limit[0]
        acc_limit_msg.linear.y = acc_limit[1]
        acc_limit_msg.angular.z = acc_limit[2]
        self.acc_limit_pub.publish(acc_limit_msg)

    def send_braking_mode(self, braking_mode_signal):
        """Publish braking mode."""
        braking_mode_msg = Bool()
        braking_mode_msg.data = braking_mode_signal[0]
        self.braking_mode_pub.publish(braking_mode_msg)

    def process_vr_head_data(self, quaternion):
        """Process VR headset quaternion data to control the robot."""
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        rospy.loginfo(f"Processing VR head quaternion: {q}")

        try:
            # Map head orientation to torso and chassis
            torso_position = [0.0, 0.0, 0.0, q[2]]  # Use z-component for torso twist
            chassis_cmd = [0.0, 0.0, q[2] * 2.0]  # Angular velocity from head yaw

            # Map head orientation to arm positions (example: mirroring head tilt)
            position_left = [q[0], q[1], 0.0, 0.0, 0.0, 0.0]  # Tilt left arm with x, y
            position_right = [-q[0], q[1], 0.0, 0.0, 0.0, 0.0]  # Mirror for right arm

            # Publish commands
            self.send_joint_state_real(position_left, position_right, torso_position)
            self.send_chassis(chassis_cmd)

            # Set reasonable velocity limits
            vel_limit_left = [1.6, 1.6, 1.6, 4.0, 4.0, 4.0]
            vel_limit_right = [1.6, 1.6, 1.6, 4.0, 4.0, 4.0]
            vel_limit_torso = [1.0, 1.6, 1.4, 1.4]
            self.send_vel_limit(vel_limit_left, vel_limit_right, vel_limit_torso)

            # Default acceleration and braking
            self.send_acc_limit([0.5, 0.5, 0.5])
            self.send_braking_mode([False])

            return "Robot: VR head data processed successfully"
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            return f"Robot: Error processing VR head data - {str(e)}"

    def head_command_callback(self, msg):
        """Handle received VR head Quaternion messages."""
        response = self.process_vr_head_data(msg)
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        if self.pipe:
            self.pipe.send(response)

def worker(pipe):
    """Worker process for ROS 1 node."""
    rospy.init_node('robot_control_node', anonymous=True)
    node = RobotControlNode(pipe)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node due to KeyboardInterrupt")
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught")

if __name__ == "__main__":
    parent_conn, child_conn = multiprocessing.Pipe()
    process = multiprocessing.Process(target=worker, args=(child_conn,))
    process.start()

    try:
        while not rospy.is_shutdown():
            if parent_conn.poll(1.0):
                print("Received response:", parent_conn.recv())
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Terminating process...")
        process.terminate()
        process.join()
        rospy.signal_shutdown("Program terminated by user")