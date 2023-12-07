
### <font color="green"> <span style="font-size:larger;"> Master Thesis </font>


<font color="green">**Author**:</font>  Rachele Nebbia Colomba  
<font color="green">**Title**:</font> Adaptive Admittance Control for Cooperative Manipulation using Dual Quaternion Representation

This work is a result of robotics research on cooperative manipulation and control for physical interaction at the *Munich Institute of Robotics and Machine Intelligence, TUM University, Munich*.  
The thesis focuses on the design of a new controller admittance controller for bimanual and cooperative manipulators, i.e Cooperative Dual Admittance controller (**CDAC**) using dual qauternion algebra. 
The control is designed to be consistent with the geometry of the Cooperative task space using the dual quaternion logarithmic mapping of wrenches and corresponding elastic displacements.  
The overall scheme consists of a stiffness adapter, cooperative admittance controller, and wrench adapter complete with an inner motion controller along with the cooperative system. 
Please have a look below for the schematic represantion of the proposed control system. 

<img src="https://github.com/rachele182/Master-Thesis/assets/75611841/02e18305-59ab-4233-a161-880aab3440e4" width="425">

The project contains two main repos: 

- **cdac** : here you can find the simulation setup and first results obtained in Matlab;

- **dual_admittance_control_ros**: here you can find readme.md files to guide trough the remaining folders containing the c++ code,ros nodes and results obtained from the experiments made on a bimanual system composed of two 7-dof robots. 

_Simulation:_ 
We demonstrate the effectiveness of our CDAC controlle by simulation studies in the presence of non-ideal conditions i.e friction and external disturbances.   
Simulations are carried on two 7 Dof Franka Emika Panda robots in **CoppeliaSim** using the MATLAB version of the DQ Robotics library. 
The scenario that we consider is a bimanual grasping task (as depicted in figure below), a common industrial scenario. The simulation is performed using the Adaptive CDAC proposed  and can be split into four phases: (1) reaching phase; (2)
grasping-lift off phase; (3) introduction of a disturbance with a constant gradient; (4) equilibrium phase.  
Instructions to run the simulation and visualize the results are in readme.txt inside the repository.

_Experiments._
The controller is tested and validated trough a bimanual grasping taksk carried out at the Munich Institute of Robotics and Machine Intelligence. Instructions and guide trought the files are in the readme.txt file inside the repository.
