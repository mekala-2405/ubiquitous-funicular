[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/LZLlpZQw)
## ROS2 Sim-to-Real Bringup: Deployment of a Custom ST3215 Robotic Arm

### Goal

Build a ROS 2 (Jazzy) package for a custom ST3215-servo robotic arm (OpenMANIPULATOR-X–like) such that the **same teleop workflow** works in **Gazebo simulation** and on the **real hardware**.

---

## Task 1 — Gazebo Simulation + ROS 2 Teleop (Simulation Deliverable)

Create a ROS 2 simulation stack where the arm is visualized using the provided **STL files** and can be moved using a ROS 2 **teleop package**.

**What must be included**

* A URDF/Xacro robot model built from the STL meshes with correct:

  * link/joint structure, axes, and frames
  * joint limits and reasonable inertias
* Gazebo bringup launch that:

  * spawns the robot cleanly
  * runs `ros2_control` controllers (joint state + trajectory controller)
  * exposes a standard control interface (e.g. `/arm_controller/joint_trajectory`)
* A teleop node/package that:

  * publishes `JointTrajectory` commands to the controller
  * supports safe joint-limit clamping
  * provides predefined **home** and **ready** poses
  * moves both gripper fingers together (or uses a mapping layer if sim gripper differs)

**Acceptance check**

* Launch Gazebo → robot appears correctly → teleop moves the simulated arm smoothly and safely.

---

## Task 2 — Real Hardware Bringup + Same Teleop (Hardware Deliverable)

Create a ROS 2 hardware bringup so the **same teleop package from Task 1** controls the **real ST3215-based arm** without changing user commands.

**What must be included**

* A hardware interface/bridge using the existing ST3215 Python driver that:

  * publishes joint states from real servos
  * accepts trajectory commands and converts them into servo commands
  * applies safety (limits, speed/step constraints, and emergency stop behavior if applicable)
* A hardware bringup launch that:

  * starts the ST3215 bridge
  * starts the same controller setup as simulation (joint states + trajectory controller)
  * keeps joint naming, limits, and home/ready poses consistent with Gazebo
* Parity verification:

  * capture a real arm joint pose
  * replay it in Gazebo
  * correct offsets/signs until the simulated end-effector closely matches the real one (within a small tolerance)

**Acceptance check**

* Launch real bringup → teleop commands move the real arm safely → replay test demonstrates sim–real consistency.

---

### Final Deliverables

1. **Gazebo simulation + teleop working** for the given arm
2. **Real arm movement using the same teleop package** (sim–real parity demonstrated)

