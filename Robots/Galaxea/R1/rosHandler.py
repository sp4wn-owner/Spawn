import multiprocessing
import traceback
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from std_msgs.msg import Bool, String, Float32, Float32MultiArray, Header
import time
import tf.transformations as tf 

class GripperControl:
    def __init__(self):
        self.header = Header()
        self.name = ""  
        self.p_des = 0.0 
        self.v_des = 0.0 
        self.kp = 0.0
        self.kd = 0.0
        self.t_ff = 0.0
        self.mode = ""    

class RobotControlNode:
    def __init__(self, pipe):
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

        # Gripper publishers
        self.left_gripper_motor_pub = rospy.Publisher(
            '/motion_control/control_gripper_left', GripperControl, queue_size=10)
        self.right_gripper_motor_pub = rospy.Publisher(
            '/motion_control/control_gripper_right', GripperControl, queue_size=10)
        self.left_gripper_position_pub = rospy.Publisher(
            '/motion_control/position_control_gripper_left', Float32, queue_size=10)
        self.right_gripper_position_pub = rospy.Publisher(
            '/motion_control/position_control_gripper_right', Float32, queue_size=10)

        # Response publisher
        self.response_pub = rospy.Publisher('robot_response', String, queue_size=10)

        # Subscribers for VR data
        rospy.Subscriber('vr_head_cmd', Quaternion, self.head_command_callback)  # Head orientation
        rospy.Subscriber('vr_controller_left', PoseStamped, self.left_controller_callback)  # Left controller pose
        rospy.Subscriber('vr_controller_right', PoseStamped, self.right_controller_callback)  # Right controller pose
        rospy.Subscriber('vr_controller_buttons', Float32MultiArray, self.controller_buttons_callback)  # Buttons and joysticks

        # Placeholder for controller data
        self.controller_data = {
            'left': {'pose': None, 'buttons': None, 'axes': None},
            'right': {'pose': None, 'buttons': None, 'axes': None}
        }
        self.head_quaternion = None

        rospy.loginfo('Robot Control Node initialized')

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

    def send_gripper_motor(self, hand, position, velocity, kp=10.0, kd=1.0, effort=0.0):
        """Publish motor control for grippers."""
        motor_msg = GripperControl()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.p_des = position
        motor_msg.v_des = velocity 
        motor_msg.kp = kp 
        motor_msg.kd = kd 
        motor_msg.t_ff = effort
        if hand == 'left':
            self.left_gripper_motor_pub.publish(motor_msg)
        elif hand == 'right':
            self.right_gripper_motor_pub.publish(motor_msg)

    def send_gripper_position(self, hand, stroke):
        """Publish position control for grippers (0 to 100 mm)."""
        position_msg = Float32()
        position_msg.data = max(0.0, min(100.0, stroke))  # Clamp between 0 and 100 mm
        if hand == 'left':
            self.left_gripper_position_pub.publish(position_msg)
        elif hand == 'right':
            self.right_gripper_position_pub.publish(position_msg)

    def head_command_callback(self, msg):
        """Handle received VR head Quaternion messages."""
        self.head_quaternion = [msg.x, msg.y, msg.z, msg.w]
        self.process_vr_data()

    def left_controller_callback(self, msg):
        """Handle left controller pose data."""
        self.controller_data['left']['pose'] = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, 
                           msg.pose.orientation.z, msg.pose.orientation.w]
        }
        self.process_vr_data()

    def right_controller_callback(self, msg):
        """Handle right controller pose data."""
        self.controller_data['right']['pose'] = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, 
                           msg.pose.orientation.z, msg.pose.orientation.w]
        }
        self.process_vr_data()

    def controller_buttons_callback(self, msg):
        """Handle controller buttons and joystick data."""
        num_buttons = 6
        num_axes = 4
        mid = len(msg.data) // 2
        
        self.controller_data['left']['buttons'] = msg.data[:num_buttons]
        self.controller_data['left']['axes'] = msg.data[num_buttons:num_buttons + num_axes]
        self.controller_data['right']['buttons'] = msg.data[mid:mid + num_buttons]
        self.controller_data['right']['axes'] = msg.data[mid + num_buttons:mid + num_buttons + num_axes]
        self.process_vr_data()

    def process_vr_data(self):
        """Process all VR data (head and controllers) to control the robot."""
        if not self.head_quaternion or not self.controller_data['left']['pose'] or not self.controller_data['right']['pose']:
            return

        try:
            # Convert head quaternion to Euler angles (roll, pitch, yaw)
            euler = tf.euler_from_quaternion(self.head_quaternion)
            head_yaw = euler[2]  # Yaw for torso/chassis rotation

            # Map head orientation to torso
            torso_position = [0.0, 0.0, head_yaw]

            # Map controller poses to arm joint positions
            left_pos = self.controller_data['left']['pose']['position']
            right_pos = self.controller_data['right']['pose']['position']
            position_left = [left_pos[0], left_pos[1], left_pos[2], 0.0, 0.0, 0.0]
            position_right = [right_pos[0], right_pos[1], right_pos[2], 0.0, 0.0, 0.0]

            # Map joystick axes to chassis movement
            if self.controller_data['left']['axes']:
                chassis_cmd = [
                    self.controller_data['left']['axes'][1] * 1.0,  # Forward/back (Y-axis)
                    self.controller_data['left']['axes'][0] * 1.0,  # Left/right (X-axis)
                    head_yaw * 2.0                          # Angular from head
                ]
            else:
                chassis_cmd = [0.0, 0.0, head_yaw * 2.0]

            # Map controller buttons to gripper control
            if self.controller_data['left']['buttons'] and self.controller_data['right']['buttons']:
                # Example: Trigger (Button 0) for position, Grip (Button 1) for motor control
                left_trigger = self.controller_data['left']['buttons'][0]  # 0.0 to 1.0
                right_trigger = self.controller_data['right']['buttons'][0]
                left_grip_btn = self.controller_data['left']['buttons'][1]
                right_grip_btn = self.controller_data['right']['buttons'][1]

                # Gripper position control (0 to 100 mm)
                left_stroke = left_trigger * 100.0  # Scale 0-1 to 0-100 mm
                right_stroke = right_trigger * 100.0
                self.send_gripper_position('left', left_stroke)
                self.send_gripper_position('right', right_stroke)

                # Gripper motor control (position-based with velocity)
                left_gripper_pos = left_stroke  # Target position in mm
                right_gripper_pos = right_stroke
                left_gripper_vel = 10.0 if left_grip_btn > 0.5 else 0.0
                right_gripper_vel = 10.0 if right_grip_btn > 0.5 else 0.0
                self.send_gripper_motor('left', left_gripper_pos, left_gripper_vel)
                self.send_gripper_motor('right', right_gripper_pos, right_gripper_vel)

            # Publish commands
            self.send_joint_state_real(position_left, position_right, torso_position)
            self.send_chassis(chassis_cmd)
            self.send_acc_limit([0.5, 0.5, 0.5])
            self.send_braking_mode([False])

            response = "Robot: VR data processed successfully"
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            response = f"Robot: Error processing VR data - {str(e)}"

        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        if self.pipe:
            self.pipe.send(response)

def worker(pipe):
    """Worker process for ROS 1 node."""
    rospy.init_node('robot_control_node', anonymous=True)
    node = RobotControlNode(pipe)
    rospy.spin()

if __name__ == "__main__":
    parent_conn, child_conn = multiprocessing.Pipe()
    process = multiprocessing.Process(target=worker, args=(child_conn,))
    process.start()

    while not rospy.is_shutdown():
        if parent_conn.poll(1.0):
            print("Received response:", parent_conn.recv())
        time.sleep(0.1)