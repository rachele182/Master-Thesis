
<div align="center">

# Adaptive Admittance Control for Cooperative Manipulation using Dual Quaternion Representation

**Author:** Rachele Nebbia Colomba

**Research-based cooperative manipulation & physical interaction** · Munich Institute of Robotics and Machine Intelligence (MIRMI), Technical University of Munich

[![ROS](https://img.shields.io/badge/ROS-Kinetic|Melodic-green?style=flat&logo=ros)](https://www.ros.org/)
[![C++](https://img.shields.io/badge/Language-C%2B%2B-blue)](https://isocpp.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-Simulink-orange)](https://www.mathworks.com/)
[![CoppeliaSim](https://img.shields.io/badge/Simulation-CoppeliaSim-green)](#)
[![DQ Robotics](https://img.shields.io/badge/Library-DQ%20Robotics-blueviolet)](https://dqrobotics.github.io/)
[![Paper](https://img.shields.io/badge/Publication-IEEE%20CDC-red)](#)

</div>

---
## Overview
This repository contains the research and implementation of a **Cooperative Dual Admittance Controller (CDAC)** developed as part of a Master's thesis in **robotics and control engineering**.

The controller is designed for **bimanual and cooperative manipulators**. It builds on **dual-quaternion algebra**, which the controller uses to represent both wrenches and elastic displacements in a way that is geometrically consistent with the cooperative task space. The complete scheme combines four blocks:

- a **stiffness adapter** for adaptive modulation of the compliant behavior,
- a **cooperative admittance controller**,
- a **wrench adapter**,
- an **inner motion controller** for the cooperative system.

The work is validated both **in simulation** and on **real hardware** (two 7-DoF Franka Emika Panda robots).
Please have a look below for the schematic represantion of the proposed control system. 

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/02e18305-59ab-4233-a161-880aab3440e4" width="425">


## Key Contributions

- Design of a **novel context-aware admittance control scheme (CDAC)** that adapts relative stiffness online to minimize internal stresses while avoiding slippage.
- Use of **dual-quaternion logarithmic mapping** so that forces and elastic displacements are represented consistently with the geometry of the cooperative task space.
- Validation on **real dual-arm hardware** (2 × Franka Emika Panda, libfranka + FCI, ROS nodes) for a **cooperative box-lifting task**.


## Repository Structure

```
.
├── CDAC/                          # Simulation setup & first results (MATLAB)
├── dual_admittance_control_ros/   # C++ implementation, ROS nodes, lab-experiment results
│                                  # (contains its own README guides)
├── rosbag/                        # Recorded experimental data
├── demo_plots/demo1_no_admittance/ # Visualization / demo plots
├── drawing_block.pdf              # Block schematic of the control system
├── setup_experiment.JPG           # Photo of the experimental setup
└── README.md
```

> `cdac` (MATLAB) = **simulation**, `dual_admittance_control_ros` (C++/ROS) = **real-robot experiments**.

## Simulation
  
The control strategy is validated under **non-ideal conditions** — friction and external disturbances — on two 7-DoF Franka Emika Panda robots in **CoppeliaSim**, using the **MATLAB version of the DQ Robotics library**.

The scenario is a **bimanual grasping task** (a common industrial setup), executed in four phases:

1. reaching,
2. grasping / lift-off,
3. introduction of a disturbance with constant gradient,
4. equilibrium.

> Instructions to run the simulation and visualize the results are in `readme.txt` inside the repository.  

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/5abf2cea-e787-473a-aa5a-d291a6fd6349" width="255"> 

## Real-Robot Experiments

The proposed framework was validated on a **real dual-arm system** (two 7-DoF Franka Emika Panda robots) at the **Munich Institute of Robotics and Machine Intelligence (MIRMI)**, using **libfranka** and the **Franka Control Interface (FCI)**. An external router connected both manipulators to the same network, enabling simultaneous control of the two arms. All controller blocks run as **ROS nodes**.

To prove the effectiveness of CDAC, a **cooperative box-lifting task** was performed in three configurations:

1. motion controller **without** the cooperative admittance loop,
2. cooperative admittance controller with **fixed gains**,
3. the proposed **CDAC with modulation of relative stiffness**.  

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/2bce3aba-cd9f-46b1-9d15-61a87dadfd79" width="405">


## Results & Impact

The three configurations are compared in terms of **applied internal stresses** and **slippage avoidance**, showing that adaptive modulation of the relative stiffness reduces internal forces while maintaining a robust grasp —

> Detailed results, plots and step-by-step guides are provided in the `readme.txt` files inside the repository.

## Tech Stack

| Area            | Tools / Libraries                                            |
|-----------------|--------------------------------------------------------------|
| Control theory  | Admittance control, cooperative task space, dual quaternions |
| Robotics        | Franka Emika Panda (7-DoF), Franka Control Interface (FCI)   |
| Software        | C++, ROS, MATLAB/Simulink, CoppeliaSim, DQ Robotics          |
| Hardware link   | libfranka, Ethernet/router network, real-time control        |

## Publication & Citation

This work was published at the **61st IEEE Conference on Decision and Control (CDC 2022)**, Cancún, Mexico, December 6–9, 2022 (pp. 107–114).

> 📄 [IEEE Xplore](https://doi.org/10.1109/CDC51059.2022.9992402) · [ResearchGate](https://www.researchgate.net/publication/367034973_Adaptive_Admittance_Control_for_Cooperative_Manipulation_using_Dual_Quaternion_Representation_and_Logarithmic_Mapping)

If you use this work, please cite:

```bibtex
@inproceedings{nebbiacolomba2022adaptive,
  title     = {Adaptive Admittance Control for Cooperative Manipulation
               using Dual Quaternion Representation and Logarithmic Mapping},
  author    = {Nebbia Colomba, Rachele and Laha, Riddhiman and
               Figueredo, Luis F. C. and Haddadin, Sami},
  booktitle = {2022 IEEE 61st Conference on Decision and Control (CDC)},
  pages     = {107--114},
  year      = {2022},
  address   = {Canc{\'u}n, Mexico},
  month     = dec,
  doi       = {10.1109/CDC51059.2022.9992402}
}
```

## License

Academic / research use. Please contact the author for reuse permissions.

---

**Rachele Nebbia Colomba** · MSc Robotics & Automation Engineering


