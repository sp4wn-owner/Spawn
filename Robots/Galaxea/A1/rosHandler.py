import multiprocessing
import traceback
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from std_msgs.msg import Bool, String, Float32, Float32MultiArray, Header
import time
import tf.transformations as tf
import math

class GripperControl:
    def __init__(self):
        self.header = Header()
        self.p_des = 0.0 
        self.v_des = 0.0 
        self.kp = 0.0
        self.kd = 0.0
        self.t_ff = 0.0
        self.mode = ""    

class RobotControlNode:
    def __init__(self, pipe):
        self.pipe = pipe

        # Publishers for all joint types
        self.left_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_arm_left', JointState, queue_size=10)
        self.right_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_arm_right', JointState, queue_size=10)
        self.torso_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_torso', JointState, queue_size=10)
        self.steering_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_steering', JointState, queue_size=10)
        self.wheel_joint_state_pub = rospy.Publisher(
            '/motion_target/target_joint_state_wheels', JointState, queue_size=10)
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
        rospy.Subscriber('vr_head_cmd', Quaternion, self.head_command_callback)
        rospy.Subscriber('vr_controller_left', PoseStamped, self.left_controller_callback)
        rospy.Subscriber('vr_controller_right', PoseStamped, self.right_controller_callback)
        rospy.Subscriber('vr_controller_buttons', Float32MultiArray, self.controller_buttons_callback)

        # Controller data storage
        self.controller_data = {
            'left': {'pose': None, 'buttons': None, 'axes': None},
            'right': {'pose': None, 'buttons': None, 'axes': None}
        }
        self.head_quaternion = None

        # Joint limits from URDF
        self.joint_limits = {
            'steer_motor_joint1': (-1.5708, 1.5708), 'steer_motor_joint2': (-1.5708, 1.5708), 
            'steer_motor_joint3': (-1.5708, 1.5708),
            'wheel_motor_joint1': (-float('inf'), float('inf')), 'wheel_motor_joint2': (-float('inf'), float('inf')), 
            'wheel_motor_joint3': (-float('inf'), float('inf')),
            'torso_joint1': (-1.1345, 1.8326), 'torso_joint2': (-2.7925, 2.5307), 
            'torso_joint3': (-1.8326, 1.5708), 'torso_joint4': (-3.0543, 3.0543),
            'left_arm_joint1': (-2.8798, 2.8798), 'left_arm_joint2': (0, 3.2289), 
            'left_arm_joint3': (-3.3161, 0), 'left_arm_joint4': (-2.8798, 2.8798), 
            'left_arm_joint5': (-1.6581, 1.6581), 'left_arm_joint6': (-2.8798, 2.8798),
            'right_arm_joint1': (-2.8798, 2.8798), 'right_arm_joint2': (0, 3.2289), 
            'right_arm_joint3': (-3.3161, 0), 'right_arm_joint4': (-2.8798, 2.8798), 
            'right_arm_joint5': (-1.6581, 1.6581), 'right_arm_joint6': (-2.8798, 2.8798),
            'left_gripper_finger_joint1': (0, 0.05), 'right_gripper_finger_joint1': (0, 0.05)
        }

        # Joint axes from URDF
        self.joint_axes = {
            'steer_motor_joint1': [0, 0, 1], 'steer_motor_joint2': [0, 0, 1], 'steer_motor_joint3': [0, 0, 1],
            'wheel_motor_joint1': [0, 1, 0], 'wheel_motor_joint2': [0, 1, 0], 'wheel_motor_joint3': [0, 1, 0],
            'torso_joint1': [0, 1, 0], 'torso_joint2': [0, 1, 0], 'torso_joint3': [0, -1, 0], 'torso_joint4': [0, 0, 1],
            'left_arm_joint1': [0, 1, 0], 'left_arm_joint2': [0, 0, -1], 'left_arm_joint3': [0, 0, -1],
            'left_arm_joint4': [1, 0, 0], 'left_arm_joint5': [0, 1, 0], 'left_arm_joint6': [1, 0, 0],
            'right_arm_joint1': [0, -1, 0], 'right_arm_joint2': [0, 0, 1], 'right_arm_joint3': [0, 0, 1],
            'right_arm_joint4': [1, 0, 0], 'right_arm_joint5': [0, -1, 0], 'right_arm_joint6': [1, 0, 0]
        }

        rospy.loginfo('Robot Control Node initialized')

    def send_joint_state(self, joint_type, joint_names, positions):
        """Publish joint states for specified joint type."""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = joint_names
        joint_state.position = positions
        if joint_type == 'left_arm':
            self.left_joint_state_pub.publish(joint_state)
        elif joint_type == 'right_arm':
            self.right_joint_state_pub.publish(joint_state)
        elif joint_type == 'torso':
            self.torso_joint_state_pub.publish(joint_state)
        elif joint_type == 'steering':
            self.steering_joint_state_pub.publish(joint_state)
        elif joint_type == 'wheels':
            self.wheel_joint_state_pub.publish(joint_state)

    def send_chassis(self, chassis_command):
        chassis_command_msg = Twist()
        chassis_command_msg.linear.x = chassis_command[0]
        chassis_command_msg.linear.y = chassis_command[1]
        chassis_command_msg.angular.z = chassis_command[2]
        self.chassis_command_pub.publish(chassis_command_msg)

    def send_acc_limit(self, acc_limit):
        acc_limit_msg = Twist()
        acc_limit_msg.linear.x = acc_limit[0]
        acc_limit_msg.linear.y = acc_limit[1]
        acc_limit_msg.angular.z = acc_limit[2]
        self.acc_limit_pub.publish(acc_limit_msg)

    def send_braking_mode(self, braking_mode_signal):
        braking_mode_msg = Bool()
        braking_mode_msg.data = braking_mode_signal[0]
        self.braking_mode_pub.publish(braking_mode_msg)

    def send_gripper_motor(self, hand, position, velocity, kp=10.0, kd=1.0, effort=0.0):
        motor_msg = GripperControl()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.p_des = position * 1000  # Convert meters to mm
        motor_msg.v_des = velocity 
        motor_msg.kp = kp 
        motor_msg.kd = kd 
        motor_msg.t_ff = effort
        if hand == 'left':
            self.left_gripper_motor_pub.publish(motor_msg)
        elif hand == 'right':
            self.right_gripper_motor_pub.publish(motor_msg)

    def send_gripper_position(self, hand, stroke):
        position_msg = Float32()
        position_msg.data = max(0.0, min(50.0, stroke * 1000))  # URDF uses 0-0.05m, scale to 0-50mm
        if hand == 'left':
            self.left_gripper_position_pub.publish(position_msg)
        elif hand == 'right':
            self.right_gripper_position_pub.publish(position_msg)

    def head_command_callback(self, msg):
        self.head_quaternion = [msg.x, msg.y, msg.z, msg.w]
        self.process_vr_data()

    def left_controller_callback(self, msg):
        self.controller_data['left']['pose'] = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, 
                           msg.pose.orientation.z, msg.pose.orientation.w]
        }
        self.process_vr_data()

    def right_controller_callback(self, msg):
        self.controller_data['right']['pose'] = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, 
                           msg.pose.orientation.z, msg.pose.orientation.w]
        }
        self.process_vr_data()

    def controller_buttons_callback(self, msg):
        num_buttons = 6
        num_axes = 4
        mid = len(msg.data) // 2
        self.controller_data['left']['buttons'] = msg.data[:num_buttons]
        self.controller_data['left']['axes'] = msg.data[num_buttons:num_buttons + num_axes]
        self.controller_data['right']['buttons'] = msg.data[mid:mid + num_buttons]
        self.controller_data['right']['axes'] = msg.data[mid + num_buttons:mid + num_buttons + num_axes]
        self.process_vr_data()

    def quaternion_to_angle(self, quat, axis):
        """Extract rotation angle around a specific axis from a quaternion."""
        q = [quat[3], quat[0], quat[1], quat[2]]  # tf uses [w, x, y, z]
        angle = 2 * math.acos(q[0])  # Total rotation angle
        sin_half = math.sin(angle / 2)
        if sin_half == 0:
            return 0.0
        # Project onto joint axis
        axis_vec = [q[1] / sin_half, q[2] / sin_half, q[3] / sin_half]
        dot = sum(a * b for a, b in zip(axis, axis_vec))
        return angle * math.copysign(1, dot) if abs(dot) > 0.5 else 0.0  # Only if aligned with axis

    def process_vr_data(self):
        if not self.head_quaternion or not self.controller_data['left']['pose'] or not self.controller_data['right']['pose']:
            return

        try:
            # Head orientation to torso
            head_euler = tf.euler_from_quaternion(self.head_quaternion)
            torso_positions = [
                0.0,  # torso_joint1 (not mapped directly)
                0.0,  # torso_joint2 (not mapped directly)
                head_euler[1],  # torso_joint3 (pitch)
                head_euler[2]   # torso_joint4 (yaw)
            ]
            torso_names = ['torso_joint1', 'torso_joint2', 'torso_joint3', 'torso_joint4']
            for i, (name, pos) in enumerate(zip(torso_names, torso_positions)):
                min_limit, max_limit = self.joint_limits[name]
                torso_positions[i] = max(min_limit, min(max_limit, pos))

            # Left arm from left controller quaternion
            left_quat = self.controller_data['left']['pose']['orientation']
            left_positions = [
                self.quaternion_to_angle(left_quat, self.joint_axes['left_arm_joint1']),
                self.quaternion_to_angle(left_quat, self.joint_axes['left_arm_joint2']),
                self.quaternion_to_angle(left_quat, self.joint_axes['left_arm_joint3']),
                self.quaternion_to_angle(left_quat, self.joint_axes['left_arm_joint4']),
                self.quaternion_to_angle(left_quat, self.joint_axes['left_arm_joint5']),
                self.quaternion_to_angle(left_quat, self.joint_axes['left_arm_joint6'])
            ]
            left_names = [f'left_arm_joint{i+1}' for i in range(6)]
            for i, name in enumerate(left_names):
                min_limit, max_limit = self.joint_limits[name]
                left_positions[i] = max(min_limit, min(max_limit, left_positions[i]))

            # Right arm from right controller quaternion
            right_quat = self.controller_data['right']['pose']['orientation']
            right_positions = [
                self.quaternion_to_angle(right_quat, self.joint_axes['right_arm_joint1']),
                self.quaternion_to_angle(right_quat, self.joint_axes['right_arm_joint2']),
                self.quaternion_to_angle(right_quat, self.joint_axes['right_arm_joint3']),
                self.quaternion_to_angle(right_quat, self.joint_axes['right_arm_joint4']),
                self.quaternion_to_angle(right_quat, self.joint_axes['right_arm_joint5']),
                self.quaternion_to_angle(right_quat, self.joint_axes['right_arm_joint6'])
            ]
            right_names = [f'right_arm_joint{i+1}' for i in range(6)]
            for i, name in enumerate(right_names):
                min_limit, max_limit = self.joint_limits[name]
                right_positions[i] = max(min_limit, min(max_limit, right_positions[i]))

            # Steering and wheels from left joystick (axes)
            steering_positions = [0.0, 0.0, 0.0]
            wheel_positions = [0.0, 0.0, 0.0]
            if self.controller_data['left']['axes']:
                axes = self.controller_data['left']['axes']
                # Steering from left-right (X-axis)
                steering_positions = [axes[0] * 1.5708] * 3  # Scale to ±π/2
                # Wheels from forward-back (Y-axis)
                wheel_positions = [axes[1] * 10.0] * 3  # Arbitrary speed scaling
            steering_names = ['steer_motor_joint1', 'steer_motor_joint2', 'steer_motor_joint3']
            wheel_names = ['wheel_motor_joint1', 'wheel_motor_joint2', 'wheel_motor_joint3']
            for i, name in enumerate(steering_names):
                min_limit, max_limit = self.joint_limits[name]
                steering_positions[i] = max(min_limit, min(max_limit, steering_positions[i]))

            # Chassis from head yaw and joystick
            chassis_cmd = [axes[1] * 1.0, axes[0] * 1.0, head_euler[2] * 2.0] if self.controller_data['left']['axes'] else [0.0, 0.0, head_euler[2] * 2.0]

            # Gripper control from triggers
            if self.controller_data['left']['buttons'] and self.controller_data['right']['buttons']:
                left_trigger = self.controller_data['left']['buttons'][0]  # 0-1
                right_trigger = self.controller_data['right']['buttons'][0]
                left_grip_btn = self.controller_data['left']['buttons'][1]
                right_grip_btn = self.controller_data['right']['buttons'][1]

                left_stroke = left_trigger * 0.05  # Scale 0-1 to 0-0.05m (URDF units)
                right_stroke = right_trigger * 0.05
                self.send_gripper_position('left', left_stroke)
                self.send_gripper_position('right', right_stroke)

                left_gripper_vel = 0.1 if left_grip_btn > 0.5 else 0.0
                right_gripper_vel = 0.1 if right_grip_btn > 0.5 else 0.0
                self.send_gripper_motor('left', left_stroke, left_gripper_vel)
                self.send_gripper_motor('right', right_stroke, right_gripper_vel)

            # Publish all joint states
            self.send_joint_state('left_arm', left_names, left_positions)
            self.send_joint_state('right_arm', right_names, right_positions)
            self.send_joint_state('torso', torso_names, torso_positions)
            self.send_joint_state('steering', steering_names, steering_positions)
            self.send_joint_state('wheels', wheel_names, wheel_positions)
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