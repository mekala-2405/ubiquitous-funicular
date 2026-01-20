#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
import sys
import termios
import tty
import select

# Configuration
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_JOINT = 'gripper_left_joint'
ARM_TOPIC = '/arm_controller/joint_trajectory'
GRIPPER_ACTION = '/gripper_controller/gripper_cmd'
STEP_SIZE = 0.05  # Radians to move per key press

msg = """
--------------------------------------------------
Control Your Arm!
--------------------------------------------------
Moving around:
   Joint 1:   W / S
   Joint 2:   A / D
   Joint 3:   R / F
   Joint 4:   T / G

Gripper:
   Open:      O
   Close:     C

X to quit
--------------------------------------------------
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        
        # Publishers & Subscribers
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self._action_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        # State initialization
        self.current_positions = {name: 0.0 for name in ARM_JOINTS}
        self.gripper_pos = 0.0
        self.joints_received = False
        
        self.get_logger().info("Waiting for joint states...")

    def joint_state_callback(self, msg):
        # Update current internal state from actual robot state
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos
            if name == GRIPPER_JOINT:
                self.gripper_pos = pos
        self.joints_received = True

    def send_arm_command(self, joint_idx, direction):
        if not self.joints_received:
            self.get_logger().warn("No joint states received yet. Cannot move.")
            return

        # Calculate target
        joint_name = ARM_JOINTS[joint_idx]
        target_pos = self.current_positions[joint_name] + (direction * STEP_SIZE)

        # Create message
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        
        point = JointTrajectoryPoint()
        # Fill all joints with current pos, update only the target joint
        point.positions = [self.current_positions[j] for j in ARM_JOINTS]
        point.positions[joint_idx] = target_pos
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 0.2 seconds duration

        traj.points.append(point)
        self.arm_pub.publish(traj)

    def send_gripper_command(self, open_grip):
        goal_msg = GripperCommand.Goal()
        # Open = 0.0, Close = max (e.g. 0.019 based on your URDF)
        goal_msg.command.position = 0.0 if open_grip else 0.015 
        goal_msg.command.max_effort = 100.0
        
        self._action_client.wait_for_server(timeout_sec=1.0)
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Gripper {'Open' if open_grip else 'Close'} sent")

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArmTeleop()

    try:
        print(msg)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            key = getKey()
            
            if key == 'w': node.send_arm_command(0, 1)  # J1 +
            elif key == 's': node.send_arm_command(0, -1) # J1 -
            elif key == 'a': node.send_arm_command(1, 1)  # J2 +
            elif key == 'd': node.send_arm_command(1, -1) # J2 -
            elif key == 'r': node.send_arm_command(2, 1)  # J3 +
            elif key == 'f': node.send_arm_command(2, -1) # J3 -
            elif key == 't': node.send_arm_command(3, 1)  # J4 +
            elif key == 'g': node.send_arm_command(3, -1) # J4 -
            elif key == 'o': node.send_gripper_command(True)  # Open
            elif key == 'c': node.send_gripper_command(False) # Close
            elif key == 'x': break

    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()