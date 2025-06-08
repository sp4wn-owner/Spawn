import math
import time
import struct
import numpy as np
import can
import tf.transformations as tf
import atexit
import sys
import json

class C_PiperForwardKinematics:
    def __init__(self, dh_is_offset=0x00):
        self.RADIAN = 180 / math.pi
        self.PI = math.pi
        self._a = [0, 0, 285.03, -21.98, 0, 0]  # Link lengths (mm)
        self._alpha = [0, -self.PI / 2, 0, self.PI / 2, -self.PI / 2, self.PI / 2]
        self._theta = [0, -self.PI * 174.22 / 180, -100.78 / 180 * self.PI, 0, 0, 0]
        self._d = [123, 0, 0, 250.75, 0, 91]  # Link offsets (mm)
        self.init_pos = [55.0, 0.0, 205.0, 0.0, 85.0, 0.0]  # xyz (mm), rpy (deg)
        if dh_is_offset == 0x01:
            self._a = [0, 0, 285.03, -21.98, 0, 0]
            self._alpha = [0, -self.PI / 2, 0, self.PI / 2, -self.PI / 2, self.PI / 2]
            self._theta = [0, -self.PI * 172.22 / 180, -102.78 / 180 * self.PI, 0, 0, 0]
            self._d = [123, 0, 0, 250.75, 0, 91]
            self.init_pos = [56.128, 0.0, 213.266, 0.0, 85.0, 0.0]
        self._a = [a / 1000 for a in self._a]
        self._d = [d / 1000 for d in self._d]
        self.init_pos[0:3] = [p / 1000 for p in self.init_pos[0:3]]
        self.init_pos[3:6] = [p * self.PI / 180 for p in self.init_pos[3:6]]

    def __MatrixToeula(self, T):
        Pos = [0.0] * 6
        Pos[0] = T[3]  # x
        Pos[1] = T[7]  # y
        Pos[2] = T[11] # z
        if T[8] < -1 + 0.0001:
            Pos[4] = self.PI / 2
            Pos[5] = 0
            Pos[3] = math.atan2(T[1], T[5])
        elif T[8] > 1 - 0.0001:
            Pos[4] = -self.PI / 2
            Pos[5] = 0
            Pos[3] = -math.atan2(T[1], T[5])
        else:
            _bt = math.atan2(-T[8], math.sqrt(T[0] * T[0] + T[4] * T[4]))
            Pos[4] = _bt
            Pos[5] = math.atan2(T[4] / math.cos(_bt), T[0] / math.cos(_bt))
            Pos[3] = math.atan2(T[9] / math.cos(_bt), T[10] / math.cos(_bt))
        return Pos

    def __MatMultiply(self, matrix1, matrix2, m=4, l=4, n=4):
        matrixOut = [0.0] * (m * n)
        for i in range(m):
            for j in range(n):
                tmp = 0.0
                for k in range(l):
                    tmp += matrix1[l * i + k] * matrix2[n * k + j]
                matrixOut[n * i + j] = tmp
        return matrixOut

    def __LinkTransformtion(self, alpha, a, theta, d):
        calpha = math.cos(alpha)
        salpha = math.sin(alpha)
        ctheta = math.cos(theta)
        stheta = math.sin(theta)
        T = [0.0] * 16
        T[0] = ctheta
        T[1] = -stheta
        T[2] = 0
        T[3] = a
        T[4] = stheta * calpha
        T[5] = ctheta * calpha
        T[6] = -salpha
        T[7] = -salpha * d
        T[8] = stheta * salpha
        T[9] = ctheta * salpha
        T[10] = calpha
        T[11] = calpha * d
        T[12] = 0
        T[13] = 0
        T[14] = 0
        T[15] = 1
        return T

    def CalFK(self, cur_j):
        _Rt = [[0.0] * 16 for _ in range(6)]
        for i in range(6):
            c_theta = cur_j[i] + self._theta[i]
            _Rt[i] = self.__LinkTransformtion(self._alpha[i], self._a[i], c_theta, self._d[i])
        R02 = self.__MatMultiply(_Rt[0], _Rt[1])
        R03 = self.__MatMultiply(R02, _Rt[2])
        R04 = self.__MatMultiply(R03, _Rt[3])
        R05 = self.__MatMultiply(R04, _Rt[4])
        R06 = self.__MatMultiply(R05, _Rt[5])
        j_pos = [
            self.__MatrixToeula(_Rt[0]),
            self.__MatrixToeula(R02),
            self.__MatrixToeula(R03),
            self.__MatrixToeula(R04),
            self.__MatrixToeula(R05),
            self.__MatrixToeula(R06)
        ]
        return j_pos

class PiperArmsControl:
    def __init__(self):
        # Joint configuration from PiperArms
        self.joint_controls = [
            {'jointName': 'left_joint1', 'lowerLimit': -2.618, 'upperLimit': 2.618, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'left_joint2', 'lowerLimit': 0, 'upperLimit': 3.14, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'left_joint3', 'lowerLimit': -2.967, 'upperLimit': 0, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'left_joint4', 'lowerLimit': -1.745, 'upperLimit': 1.745, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'left_joint5', 'lowerLimit': -1.22, 'upperLimit': 1.22, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'left_joint6', 'lowerLimit': -2.0944, 'upperLimit': 2.0944, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'left_joint7', 'lowerLimit': -0.035, 'upperLimit': 0, 'axis': [0, 1, 0], 'isGripper': True, 'isPrismatic': True},
            {'jointName': 'left_joint8', 'lowerLimit': -0.035, 'upperLimit': 0, 'axis': [0, -1, 0], 'isGripper': True, 'isPrismatic': True},
            {'jointName': 'right_joint1', 'lowerLimit': -2.618, 'upperLimit': 2.618, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'right_joint2', 'lowerLimit': 0, 'upperLimit': 3.14, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'right_joint3', 'lowerLimit': -2.967, 'upperLimit': 0, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'right_joint4', 'lowerLimit': -1.745, 'upperLimit': 1.745, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'right_joint5', 'lowerLimit': -1.22, 'upperLimit': 1.22, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'right_joint6', 'lowerLimit': -2.0944, 'upperLimit': 2.0944, 'axis': [0, 0, 1], 'isPrismatic': False},
            {'jointName': 'right_joint7', 'lowerLimit': -0.035, 'upperLimit': 0, 'axis': [0, 1, 0], 'isGripper': True, 'isPrismatic': True},
            {'jointName': 'right_joint8', 'lowerLimit': -0.035, 'upperLimit': 0, 'axis': [0, -1, 0], 'isGripper': True, 'isPrismatic': True}
        ]

        # Link lengths from PiperArms
        self.link_lengths = {
            'joint2': 0.285,  # Upper arm
            'joint4': 0.2518, # Forearm
            'joint6': 0.091   # Wrist to gripper base
        }
        self.scale_factor = 1.0
        self.initial_robot_position = [0, 1.5, 0]
        self.joint_values = {control['jointName']: 0 for control in self.joint_controls}
        self.delta_time = 0.016
        self.debug = True

        # Controller offsets and initial positions
        self.controller_offsets = {'left': [0, 0, 0], 'right': [0, 0, 0]}
        self.initial_controller_positions_set = {'left': False, 'right': False}
        self.initial_controller_positions = {'left': None, 'right': None}

        # FK instances
        self.fk_left = C_PiperForwardKinematics(dh_is_offset=0x00)
        self.fk_right = C_PiperForwardKinematics(dh_is_offset=0x00)

        # FK verification thresholds
        self.pos_error_threshold = 0.01
        self.orient_error_threshold = 0.1

        # CAN interfaces
        try:
            self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
            self.can1 = can.interface.Bus(channel='can1', bustype='socketcan', bitrate=500000)
            print("CAN interfaces initialized: can0, can1")
        except Exception as e:
            print(f"Failed to initialize CAN interfaces: {e}")
            raise

        # Register cleanup function to reset joint angles on exit
        atexit.register(self._reset_joints)

    def _reset_joints(self):
        """Reset all joint angles to 0 for both arms on script termination."""
        try:
            for hand in ['left', 'right']:
                # Prepare zero angles for 6 arm joints + 2 gripper joints
                zero_angles = [0.0] * 8
                bus = self.can0 if hand == 'left' else self.can1
                arb_id = 0x100 if hand == 'left' else 0x101
                # Pack 8 joint angles as 32 bytes (8 floats)
                data = b''.join(struct.pack('!f', angle) for angle in zero_angles)
                msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
                bus.send(msg)
                print(f"Reset {hand} joint angles to zero: {zero_angles}")
                # Update internal joint values
                for i in range(1, 9):
                    self.joint_values[f"{hand}_joint{i}"] = 0.0
        except can.CanError as e:
            print(f"Failed to reset {hand} joint angles: {e}")
        finally:
            # Shutdown CAN interfaces
            try:
                self.can0.shutdown()
                self.can1.shutdown()
                print("CAN interfaces shut down")
            except Exception as e:
                print(f"Error shutting down CAN interfaces: {e}")

    def send_joint_angles(self, hand, joint_angles):
        """Send joint angles over CAN."""
        bus = self.can0 if hand == 'left' else self.can1
        arb_id = 0x100 if hand == 'left' else 0x101
        # Pack 8 joint angles as 32 bytes (8 floats)
        data = b''.join(struct.pack('!f', angle) for angle in joint_angles)
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        try:
            bus.send(msg)
            if self.debug:
                print(f"Sent {hand} joint angles over {bus.channel}: {', '.join([f'{a:.3f}' for a in joint_angles])}")
        except can.CanError as e:
            print(f"Failed to send {hand} joint angles: {e}")

    def compute_arm_ik(self, hand, target_pos, delta_time, target_orientation=None):
        prefix = hand
        link_lengths = {
            'upperArm': self.link_lengths['joint2'] * self.scale_factor,
            'forearm': self.link_lengths['joint4'] * self.scale_factor,
            'wristToGripper': self.link_lengths['joint6'] * self.scale_factor
        }
        max_reach = link_lengths['upperArm'] + link_lengths['forearm'] + link_lengths['wristToGripper']

        if not all(np.isfinite(target_pos)):
            if self.debug:
                print(f"{prefix} computeArmIK: Invalid target position, skipping")
            return [0] * 6

        base_pos = np.array(self.initial_robot_position)
        base_rotation = tf.euler_matrix(-np.pi / 2, 0, np.pi / 2, 'rxyz')
        target = np.array(target_pos) - base_pos
        target = np.dot(np.linalg.inv(base_rotation[:3, :3]), target)
        if self.debug:
            print(f"{prefix} target in base frame: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}")

        target_dist = np.linalg.norm(target)
        if target_dist > max_reach:
            scale = max_reach / target_dist
            target *= scale
            if self.debug:
                print(f"{prefix} target clamped to max reach: {max_reach:.3f}, scale={scale:.3f}")

        arm_joints = [f"{prefix}_joint{i+1}" for i in range(6)]
        joint_controls = [c for c in self.joint_controls if c['jointName'].startswith(f"{prefix}_") and not c.get('isGripper', False)]
        joint_updates = {}
        joint_max_delta = {
            f"{prefix}_joint1": 3.5 * delta_time,
            f"{prefix}_joint2": 3.5 * delta_time,
            f"{prefix}_joint3": 3.0 * delta_time,
            f"{prefix}_joint4": 4.0 * delta_time,
            f"{prefix}_joint5": 4.0 * delta_time,
            f"{prefix}_joint6": 4.0 * delta_time
        }

        # Compute yaw (joint1) with offset
        joint1_control = joint_controls[0]
        xy_dist = np.sqrt(target[0]**2 + target[2]**2 + 1e-10)
        yaw_offset = -0.3 if hand == 'left' else 0.3  # Yaw offset: -0.3 for left, +0.3 for right
        yaw = np.arctan2(target[1], xy_dist)
        if hand == 'left':
            yaw = -yaw + yaw_offset
        else:
            yaw = yaw + yaw_offset
        joint_updates[arm_joints[0]] = max(joint1_control['lowerLimit'], min(joint1_control['upperLimit'], yaw))
        if self.debug:
            print(f"{prefix}_joint1 (yaw with offset {yaw_offset:.3f}): {joint_updates[arm_joints[0]]:.3f} rad, xyDist={xy_dist:.3f}")

        # Transform target to joint2 frame
        yaw_angle = joint_updates[arm_joints[0]]
        cos_yaw = np.cos(-yaw_angle)
        sin_yaw = np.sin(-yaw_angle)
        target_in_joint2 = np.array([
            target[0] * cos_yaw + target[1] * (-sin_yaw if hand == 'right' else sin_yaw),
            -target[0] * sin_yaw + target[1] * (cos_yaw if hand == 'right' else cos_yaw),
            target[2]
        ])
        if self.debug:
            print(f"{prefix} target in joint2 frame: x={target_in_joint2[0]:.3f}, y={target_in_joint2[1]:.3f}, z={target_in_joint2[2]:.3f}")

        # Compute pitch (joint2)
        joint2_control = joint_controls[1]
        pitch = np.arctan2(-target_in_joint2[2], target_in_joint2[0]) + 1.57
        joint_updates[arm_joints[1]] = max(joint2_control['lowerLimit'], min(joint2_control['upperLimit'], pitch))
        if self.debug:
            print(f"{prefix}_joint2 (pitch after +1.57): {joint_updates[arm_joints[1]]:.3f} rad")

        # Transform target to joint3 frame and compute elbow (joint3)
        pitch_angle = joint_updates[arm_joints[1]] - 1.5708
        wrist_target = target - np.array([0, 0, link_lengths['wristToGripper']])
        wrist_target = np.array([
            wrist_target[0] * cos_yaw + wrist_target[1] * (-sin_yaw if hand == 'right' else sin_yaw),
            -wrist_target[0] * sin_yaw + wrist_target[1] * cos_yaw,
            wrist_target[2]
        ])
        cos_pitch = np.cos(-pitch_angle)
        sin_pitch = np.sin(-pitch_angle)
        wrist_target = np.array([
            wrist_target[0] * cos_pitch + wrist_target[2] * sin_pitch,
            wrist_target[1],
            -wrist_target[0] * sin_pitch + wrist_target[2] * cos_pitch
        ])
        if self.debug:
            print(f"{prefix} wrist target in joint3 frame: x={wrist_target[0]:.3f}, y={wrist_target[1]:.3f}, z={wrist_target[2]:.3f}")

        joint3_control = joint_controls[2]
        wrist_dist = np.linalg.norm(wrist_target)
        wrist_dist_xz = np.sqrt(wrist_target[0]**2 + wrist_target[2]**2 + 1e-10)
        max_wrist_dist = link_lengths['upperArm'] + link_lengths['forearm'] - 0.01
        if wrist_dist > max_wrist_dist:
            wrist_target *= max_wrist_dist / wrist_dist
            if self.debug:
                print(f"{prefix} wrist target clamped: x={wrist_target[0]:.3f}, y={wrist_target[1]:.3f}, z={wrist_target[2]:.3f}")

        cos_elbow = np.clip(
            (link_lengths['upperArm']**2 + wrist_dist_xz**2 - link_lengths['forearm']**2) /
            (2 * link_lengths['upperArm'] * wrist_dist), -0.999, 0.999
        )
        elbow_angle = np.arccos(cos_elbow)
        if wrist_target[2] < 0:
            elbow_angle = -elbow_angle
        joint_updates[arm_joints[2]] = max(joint3_control['lowerLimit'], min(joint3_control['upperLimit'], elbow_angle - 0.74175))
        if self.debug:
            print(f"{prefix}_joint3 (elbow after -0.74): {joint_updates[arm_joints[2]]:.3f} rad")

        # Compute wrist joints (joint4, joint5, joint6) for orientation
        if target_orientation and all(np.isfinite(target_orientation)):
            base_quat = tf.quaternion_from_euler(-np.pi / 2, 0, np.pi / 2, 'rxyz')
            base_quat_inv = tf.quaternion_conjugate(base_quat)
            target_quat = tf.quaternion_multiply(base_quat_inv, target_orientation)
            yaw_quat = tf.quaternion_from_euler(joint_updates[arm_joints[0]], 0, 0, 'rzyx')
            pitch_quat = tf.quaternion_from_euler(0, joint_updates[arm_joints[1]] - 1.57, 0, 'ryzx')
            elbow_quat = tf.quaternion_from_euler(0, joint_updates[arm_joints[2]] + 0.74175, 0, 'ryzx')
            arm_quat = tf.quaternion_multiply(tf.quaternion_multiply(yaw_quat, pitch_quat), elbow_quat)
            wrist_quat = tf.quaternion_multiply(tf.quaternion_conjugate(arm_quat), target_quat)
            wrist_euler = tf.euler_from_quaternion(wrist_quat, 'xzy')
            wrist_euler = list(wrist_euler)  # Convert to list for modification
            wrist_euler[1] += 1.8708 if hand == 'left' else 2.1708  # Apply wrist offset
            wrist_angles = [-wrist_euler[1], wrist_euler[2], wrist_euler[0]]  # Map to joint4, joint5, joint6
            wrist_joint_controls = [joint_controls[i] for i in range(3, 6)]
            for i, (angle, control) in enumerate(zip(wrist_angles, wrist_joint_controls)):
                joint_updates[arm_joints[3 + i]] = max(control['lowerLimit'], min(control['upperLimit'], angle))
                if self.debug:
                    print(f"{arm_joints[3 + i]} (wrist): {joint_updates[arm_joints[3 + i]]:.3f} rad")
        else:
            joint_updates[arm_joints[3]] = 0
            joint_updates[arm_joints[4]] = 0
            joint_updates[arm_joints[5]] = 0
            if self.debug:
                print(f"{prefix} wrist angles set to 0 (no valid orientation)")

        # Apply smoothing to joint angles
        theta = []
        for joint_name in arm_joints:
            angle = joint_updates[joint_name]
            current = self.joint_values.get(joint_name, 0)
            if not np.isfinite(angle):
                print(f"Invalid angle for {joint_name}: {angle}")
                theta.append(current)
                continue
            delta = angle - current
            max_delta_for_joint = joint_max_delta.get(joint_name, 1.0 * delta_time)
            if abs(delta) > max_delta_for_joint:
                angle = current + np.sign(delta) * max_delta_for_joint
            theta.append(angle)
            self.joint_values[joint_name] = angle

        if self.debug:
            print(f"{prefix} final joint angles: {', '.join([f'{a:.3f}' for a in theta])}")
        return theta

    def verify_fk(self, hand, joint_angles, target_pos, target_quat):
        fk = self.fk_left if hand == 'left' else self.fk_right
        base_pos = np.array(self.initial_robot_position)
        base_rotation = tf.euler_matrix(-np.pi / 2, 0, np.pi / 2, 'rxyz')
        target_pos_base = np.dot(np.linalg.inv(base_rotation[:3, :3]), np.array(target_pos) - base_pos)

        j_pos = fk.CalFK(joint_angles)
        fk_pose = j_pos[-1]
        fk_pos = np.array(fk_pose[:3])
        fk_euler = np.array(fk_pose[3:])

        pos_error = np.linalg.norm(fk_pos - target_pos_base)
        valid_pos = pos_error < self.pos_error_threshold

        valid_orient = True
        orient_error = 0.0
        if target_quat and all(np.isfinite(target_quat)):
            base_quat = tf.quaternion_from_euler(-np.pi / 2, 0, np.pi / 2, 'rxyz')
            base_quat_inv = tf.quaternion_conjugate(base_quat)
            target_quat_base = tf.quaternion_multiply(base_quat_inv, target_quat)
            fk_quat = tf.quaternion_from_euler(fk_euler[0], fk_euler[1], fk_euler[2], 'rzyx')
            quat_diff = tf.quaternion_multiply(target_quat_base, tf.quaternion_conjugate(fk_quat))
            angle_diff = 2 * np.arccos(np.clip(quat_diff[3], -1.0, 1.0))
            orient_error = abs(angle_diff)
            valid_orient = orient_error < self.orient_error_threshold

        if self.debug:
            print(f"{hand} FK verification:")
            print(f"  FK position: x={fk_pos[0]:.3f}, y={fk_pos[1]:.3f}, z={fk_pos[2]:.3f}")
            print(f"  Target position: x={target_pos_base[0]:.3f}, y={target_pos_base[1]:.3f}, z={target_pos_base[2]:.3f}")
            print(f"  Position error: {pos_error:.3f} m (threshold: {self.pos_error_threshold})")
            if target_quat and all(np.isfinite(target_quat)):
                print(f"  FK orientation (rpy): roll={fk_euler[0]:.3f}, pitch={fk_euler[1]:.3f}, yaw={fk_euler[2]:.3f}")
                target_euler = tf.euler_from_quaternion(target_quat_base, 'rzyx')
                print(f"  Target orientation (rpy): roll={target_euler[0]:.3f}, pitch={target_euler[1]:.3f}, yaw={target_euler[2]:.3f}")
                print(f"  Orientation error: {orient_error:.3f} rad (threshold: {self.orient_error_threshold})")
            print(f"  Valid: position={valid_pos}, orientation={valid_orient}")

        return valid_pos and valid_orient

    def process_vr_data(self, controller_data):
        response = "Success"
        try:
            for hand in ['left', 'right']:
                controller = controller_data.get(hand, {})
                prefix = hand
                if not controller.get('pose', {}).get('position') or not all(np.isfinite(controller['pose']['position'])):
                    if self.debug:
                        print(f"{hand} controller position invalid, skipping arm update")
                    continue

                if not self.initial_controller_positions_set[hand]:
                    self.initial_controller_positions[hand] = controller['pose']['position'].copy()
                    self.initial_controller_positions_set[hand] = True
                    for control in self.joint_controls:
                        if control['jointName'].startswith(f"{prefix}_"):
                            self.joint_values[control['jointName']] = 0
                            if self.debug:
                                print(f"Reset {control['jointName']} to 0 on first {hand} controller data")
                    if self.debug:
                        pos = self.initial_controller_positions[hand]
                        print(f"{hand} initial position set: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

                buttons = controller.get('buttons', [])
                if buttons and buttons[1] > 0.5 and self.initial_controller_positions_set[hand]:
                    initial_pos = np.array(self.initial_controller_positions[hand])
                    current_pos = np.array(controller['pose']['position'])
                    self.controller_offsets[hand] = initial_pos - current_pos
                    if self.debug:
                        print(f"{hand} offset updated: x={self.controller_offsets[hand][0]:.3f}, y={self.controller_offsets[hand][1]:.3f}, z={self.controller_offsets[hand][2]:.3f}")

                adjusted_pos = np.array(controller['pose']['position']) + self.controller_offsets[hand]
                orientation = controller['pose']['orientation'] if controller['pose'].get('orientation') else None

                joint_angles = self.compute_arm_ik(hand, adjusted_pos, self.delta_time, orientation)

                if not self.verify_fk(hand, joint_angles, adjusted_pos, orientation):
                    print(f"{hand} FK verification failed, skipping CAN transmission")
                    continue

                trigger = buttons[0] if buttons else 0
                is_open = trigger < 0.5
                joint7_control = next(c for c in self.joint_controls if c['jointName'] == f"{prefix}_joint7")
                joint8_control = next(c for c in self.joint_controls if c['jointName'] == f"{prefix}_joint8")
                joint7_value = joint7_control['lowerLimit'] if is_open else joint7_control['upperLimit']
                joint8_value = joint8_control['lowerLimit'] if is_open else joint8_control['upperLimit']
                self.joint_values[f"{prefix}_joint7"] = joint7_value
                self.joint_values[f"{prefix}_joint8"] = joint8_value

                joint_positions = joint_angles + [joint7_value, joint8_value]
                self.send_joint_angles(hand, joint_positions)
                if self.debug:
                    print(f"Set {prefix}_joint7 to {joint7_value:.3f}")
                    print(f"Set {prefix}_joint8 to {joint8_value:.3f}")

        except Exception as e:
            print(f"Error processing VR data: {e}")
            response = f"Error: {e}"
        return response

def main():
    control = PiperArmsControl()
    try:
        for line in sys.stdin:
            try:
                # Parse JSON data from stdin
                controller_data = json.loads(line.strip())
                response = control.process_vr_data(controller_data)
                print(f"Response: {response}", flush=True)
                time.sleep(control.delta_time)
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}", flush=True)
            except Exception as e:
                print(f"Error processing input: {e}", flush=True)
    except KeyboardInterrupt:
        print("Script interrupted, resetting joints and shutting down...")
        # The atexit handler will take care of resetting joints and shutting down CAN interfaces

if __name__ == "__main__":
    main()