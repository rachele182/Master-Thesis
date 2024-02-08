# Read Me
<font color="green">**Author**:</font>  Rachele Nebbia Colomba  
<font color="green">**Description of contents**:</font> 

Here after you can find a brief description of the contents of the C++ and ROS files used for the experiments with real Franka Emika Panda Robots. The controller was designed by using ros_noetic version and FRANKA_ROS to interface with the robots.  
Please refer to the following description of directories:
1. **panda controllers**:
it contains all the files needed to run ros and interface with the two arms robot, specifically:  
_src_: contain .cpp files.
_include_: header of cpp files with class definition and variables.
_launch_: launch ros files for dual controller, admittance loop, and trajectory generator;
_msg_: customed ros mgs for publisher and subscriber nodes. 
3. **rosbag**: it contains the recorded data from different grasping tasks experiments and the script used to plot those data in matlab eg:  
_extrac_data.m_  
4. **demoplots**: it contains all the visual results of the threee different experiments carried out in terms of .svg images and .mov videos.

A brief guide trough the **main** files and the workflow is reported below:
- **Single arm** framework:
  - motion_control_dq.cpp: contains the code for the inner motion controller of a single arm, based on dynamics inversion using DQ algebra;
  - impedance_loop_dq.cpp: contains the outer admittance loop used to computed the compliant trajectory based on the estimated external forces; 
  - commands_dq.cpp: contains the code used to compute fast and editable nominal trajectories w.r.t to dual qauternion variables (based on minimun jerk interpolation);
These files were used to test and validate the approach easily on a single arm.
- **Dual arm** framework:
  - dual_arm_control.cpp: contains the code for the inner motion controller of the coupled bimanual system, based on dynamics inversion using DQ algebra;
  - dual_impedance_loop.cpp: contains the outer admittance loop used to computed the compliant trajectory (in terms of relative and absolute poses of the two arms) based on the estimated external forces;
  - modulator.cpp: contains the modulation of relative stiffness gains for the outer loop, used to guarantee grasping feasibility as well as safe interaction.
  - commands_dq.cpp: contains DEMOS trajectories used to test different grasping tasks of the bimanual system.

Idea to run the grasping task dual-arm experiment:  
--- launch ROS controller with _dual_arm.launch_;   
--- start rose nodes for: _dual_impedance_loop_ and _modulator.cpp_;  
--- start rose node commands_dual and choose the preferred DEMO.   


****NB**** Please note that **CMake.txt** represents the configuration file used to determine the ros settings and the nodes being launched. 
This need to be edited carefully and adapted for local use. 
