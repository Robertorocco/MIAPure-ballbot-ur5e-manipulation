# 🤖 BallBot Dynamic Pick & Place with HTN Planning and MoveIt

> A project for the control and planning of a BallBot equipped with a UR5e manipulator.  
> Implements robust control strategies (LQR-PI, MRAC) for stabilization and uses HTN planning and MoveIt for pick & place tasks in a Blocksworld environment.

---

## 📖 About This Project

This repository contains the project for the **"Field & Service Robotics"** course at the **University of Naples Federico II**.  
The goal is to develop a complete control and planning architecture for a complex mobile manipulator: a **UR5e robotic arm** mounted on an intrinsically unstable **BallBot**.

### The project addresses key challenges:
1. **Dynamic Stabilization:** Actively balancing the BallBot, which behaves like an inverted pendulum.  
2. **Dynamic Coupling:** Managing the coupling effects and disturbances caused by the manipulator’s movement.  
3. **Task Execution:** Enabling autonomous navigation and "pick & place" tasks in a cluttered Blocksworld environment.

The system integrates high-fidelity dynamic modeling, advanced control strategies (LQR, MRAC), and a hybrid planning architecture combining **HTN symbolic planning** with **ROS 2 MoveIt motion planning**.

---

## ✨ Core Features

- **Dynamic Stabilization:** Implements model-based control strategies (**Cascade LQR-PI**, **MRAC**) for robust balance control.  
- **Dynamic Disturbance Rejection:** Uses a **Hybrid Compensated LQR-PI Controller** that computes equilibrium tilt angles to counteract manipulator-induced disturbances.  
- **Autonomous Navigation:** Employs **RRT\*** for collision-free path generation and smooth trapezoidal velocity profiles for the motion controller.  
- **Symbolic Task Planning:** Utilizes a **Hierarchical Task Network (HTN)** planner to generate high-level symbolic actions, e.g.:
  ```python
  [('unstack', 'block_a', 'block_c'), ('putdown', 'block_a')]
