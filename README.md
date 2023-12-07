
### <font color="green"> <span style="font-size:larger;"> Master Thesis </font>


<font color="green">**Author**:</font>  Rachele Nebbia Colomba  
<font color="green">**Title**:</font> Adaptive Admittance Control for Cooperative Manipulation using Dual Quaternion Representation

This work is a result of robotics research on cooperative manipulation and control for physical interaction at the *Munich Institute of Robotics and Machine Intelligence, TUM University, Munich*.  
The thesis focuses on the design of a new controller admittance controller for bimanual and cooperative manipulators, i.e Cooperative Dual Admittance controller (**CDAC**) using dual quaternion algebra. 
The control is designed to be consistent with the geometry of the Cooperative task space using the dual quaternion logarithmic mapping of wrenches and corresponding elastic displacements.  
The overall scheme consists of a stiffness adapter, cooperative admittance controller, and wrench adapter complete with an inner motion controller along with the cooperative system. 
Please have a look below for the schematic represantion of the proposed control system. 

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/02e18305-59ab-4233-a161-880aab3440e4" width="425">

The project contains two main repos: 

- **cdac** : here you can find the simulation setup and first results obtained in Matlab;

- **dual_admittance_control_ros**: here you can find readme.md files to guide trough the remaining folders containing the c++ code,ros nodes and results obtained from the experiments made on a bimanual system composed of two 7-dof robots. 

_Simulation:_ 
We demonstrate the effectiveness of our CDAC controller by simulation studies in the presence of non-ideal conditions i.e friction and external disturbances.   
Simulations are carried on two 7 Dof Franka Emika Panda robots in **CoppeliaSim** using the MATLAB version of the DQ Robotics library. 
The scenario that we consider is a bimanual grasping task (as depicted in figure below), a common industrial scenario. The simulation is performed using the Adaptive CDAC proposed  and can be split into four phases: (1) reaching phase; (2)
grasping-lift off phase; (3) introduction of a disturbance with a constant gradient; (4) equilibrium phase.  
Instructions to run the simulation and visualize the results are in readme.txt inside the repository.

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/5abf2cea-e787-473a-aa5a-d291a6fd6349" width="255">

_Experiments._
The proposed framework presented in \ref{subsec:CDAC_design} was validated trough some cooperative tasks experiments on a real dual-arm system composed of two 7Dof Franka Emika Panda robots.  
The experiments were carried out at *Munich Istitute of Robotics* using **libfranka** and **Franka Control Interface** system to communicate with the robots. An external router device was used to connect the manipulators to the same network and allow the combined control of the two arms simultaneously. All the blocks componing the controller were implemented as nodes using Robot Operating System (ROS).  
An overview of the experiment setups is shown in the figure.  
To prove the effectiveness of **CDAC** we performed a cooperative lifting task of a box performed in three different ways.   
Firstly, the task is performed using the motion controller without the cooperative admittance loop; then the lifting task is performed using the cooperative admittance controller with fixed gains; finally we test our CDAC with modulation of the relative stiffness.  
The results in terms of applied internal stresses and slippage avoidance will be compared in the study.   
Instructions and guide trought the files are in the readme.txt file inside the repository.

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/2bce3aba-cd9f-46b1-9d15-61a87dadfd79" width="405">
