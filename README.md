# 🦾 GP255 Industrial Robot — Kinematics, Trajectory Planning & Simscape

> **Mohammed Emad Said Soliman** | UHCI4H  
> Mechatronics Engineering BSc. — University of Debrecen  
> Faculty of Engineering | Department of Electrical Engineering and Mechatronics  
> *Cyber-Physical Systems (CPS) — 2025/2026/II. Semester*

---

## 📋 Overview

This project implements **forward/inverse kinematics, circular trajectory planning, joint torque analysis, and full Simscape physical simulation** for the **Kawasaki GP255 6-DOF industrial robot arm** using the **MATLAB Robotics Toolbox** and **Simulink/Simscape Multibody**.

The robot model was imported from **OnShape CAD** directly into Simulink via `smexportonshape`, creating a physics-based simulation with real joint constraints and geometry.

---

## 🤖 Robot: Kawasaki GP255

| Property | Value |
|----------|-------|
| Type | 6-DOF articulated industrial robot |
| Joints | 6 revolute joints (J1–J6) |
| End-effector | Body6 |
| Application | Industrial pick-and-place, welding, assembly |
| Model source | OnShape CAD → Simscape Multibody |

🔗 **OnShape Model:** [View on OnShape](https://cad.onshape.com/documents/08b071bf0234bff7ace22b17/w/4c3e73d7dddde8733aaab38a/e/aab74ebe8122bfda9ac3ac4f)

---

## 🎯 Project Tasks

### 1. Forward Kinematics
- Import GP255 robot using `importrobot('GP255')`
- Compute end-effector pose at home configuration using `getTransform()`
- Verify joint angle → end-effector position mapping

### 2. Inverse Kinematics
- Set up IK solver using `inverseKinematics()`
- Target position: `[0.5, 0.3, 1.2]` meters
- Solve for joint angles and verify end-effector position

### 3. Circular Trajectory Planning
- Plan a **circular trajectory** (radius = 0.2m) in the XY plane
- Uses a **trapezoidal velocity profile** (theta ramp)
- Duration: 2 seconds, 201 time steps
- Computes joint positions `q_traj` and velocities `dq_traj` via IK at each step

### 4. Joint Torque Analysis (Inverse Dynamics)
- Computes required **joint torques** using `inverseDynamics()`
- Reports **peak torque** and **average torque** per joint
- Validates motor selection for each joint

### 5. Simscape Multibody Simulation
- Robot imported via `smexportonshape()` from OnShape
- Full physics simulation with joint constraints and contact
- Animated in Simscape Mechanics Explorer

---

## 📁 File Structure

```
├── 🤖 MATLAB Scripts
│   ├── GP255X.m              — Main script: FK, IK, trajectory, torque analysis
│   ├── GP255_DataFile.m      — Robot data file (auto-generated from import)
│   ├── GP25_DataFile.m       — Alternative data file
│   ├── GP2552.m              — OnShape import script (smexportonshape)
│   └── GP251.xml             — Robot XML description
│
├── 🔧 Simulink / Simscape
│   └── GP255.slx             — Full Simscape Multibody simulation model
│
├── 📐 CAD Files (.step)
│   ├── _BASE_AXIS_SW0001.step    — Robot base axis
│   ├── _B_AXIS_SW0001.step       — B-axis (J1)
│   ├── _L_AXIS_SW0002.step       — L-axis (J2)
│   ├── _R_AXIS_SW0001.step       — R-axis (J3)
│   ├── _S_AXIS_SW0001.step       — S-axis (J4)
│   ├── _T_AXIS_SW0001.step       — T-axis (J5)
│   └── _U_AXIS_SW0001.step       — U-axis (J6)
│
├── 📄 Report
│   └── UHCI4H.pdf            — Full assignment report
│
└── on shape link.txt         — Link to OnShape CAD model
```

---

## 🚀 How to Run

### Requirements
- MATLAB R2024a or later
- Robotics System Toolbox
- Simulink + Simscape Multibody

### Steps
1. Clone the repository
2. In MATLAB, add the folder to path
3. Run `GP255X.m` for kinematics + trajectory + torque analysis
4. Open `GP255.slx` in Simulink for Simscape simulation

### To re-import from OnShape:
```matlab
assemblyURL = "https://cad.onshape.com/documents/08b071bf0234bff7ace22b17/...";
exportedModel = smexportonshape(assemblyURL);
smimport(exportedModel);
```

---

## 📊 Trajectory

The circular trajectory is defined as:

```
θ(t) = 2π(t/T − sin(2πt/T)/(2π))   [trapezoidal velocity profile]
x(t) = x₀ + r·cos(θ(t))
y(t) = y₀ + r·sin(θ(t))
z(t) = z₀  [constant height]
```

Where: `x₀=0.5m`, `y₀=0.3m`, `z₀=1.2m`, `r=0.2m`, `T=2s`

---

*Mechatronics Engineering BSc. — University of Debrecen, 2026*
