
### <font color="green"> <span style="font-size:larger;"> Master Thesis </font>


<font color="green">**Author**:</font>  Rachele Nebbia Colomba  
<font color="green">**Title**:</font> Adaptive Admittance Control for Cooperative Manipulation using Dual Quaternion Representation

This work is a result of robotics research on cooperative manipulation and control for physical interaction at the *Munich Institute of Robotics and Machine Intelligence, TUM University, Munich*.  
The thesis focuses on the design of a new controller admittance controller for bimanual and cooperative manipulators, i.e Cooperative Dual Admittance controller (**CDAC**) using dual qauternion algebra. 
The control is designed to be consistent with the geometry of the Cooperative task space using the dual quaternion logarithmic mapping of wrenches and corresponding elastic displacements.  
The overall scheme consists of a stiffness adapter, cooperative admittance controller, and wrench adapter complete with an inner motion controller along with the cooperative system. 
Please have a look below for the schematic represantion of the proposed control system. 

The project contains two main repos: 

- **cdac** : here you can find the simulation setup and first results obtained in Matlab;

- **dual_admittance_control_ros**: here you can find readme.md files to guide trough the remaining folders containing the c++ code,ros nodes and results obtained from the experiments made on a bimanual system composed of two 7-dof robots. 

_Simulation:_ 
The CDAC is tested on a grasping task for two 7dof Panda robots by using CoppeliaSim via remApi functions in Matlab. Instructions to run the simulation and visualize the results are in readme.txt inside the repository.

_Experiments._
The controller is tested and validated trough a bimanual grasping taksk carried out at the Munich Institute of Robotics and Machine Intelligence. Instructions and guide trought the files are in the readme.txt file inside the repository.
