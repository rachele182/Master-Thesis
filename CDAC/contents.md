### <font color="green"> <span style="font-size:larger;"> Contents of CDAC: </font> </span>

Here after you can find an overview of the contents of the folder that includes all the scripts,functions and Vrep files necessary for the full simulation.   

- **Adapting Stiffness**:  
  This folder contains all the scripts files needed for the simulation of the full task. In *data* and *plots* folder you can find some figures and script to visualize some results of previous simulations.
  Please refer to **readme.txt** file for insructions on how to run.  

  **Files**: 
    - init.m = initialization file to setup the environment;
    - grasping_task.m = main simulation file;
    - grasping_task_absolute_adaptation.m = main simulation file, both relative and absolute stiffness are changed during the task;
    - abs_stiff_adapter.m, rel_stiff_adapter.m = modulator of admittance gains;
    - grasp_traj_ad.m, gripper_traj.m = scripts containing the minimum jerk interpolation to compute the nominal trajectory;
    - admittance_control.m = outer admittance loop to compute compliant trajectory;
    - add_load.m, wrench_ext_grasp_task.m = scripts used to model the disturbance and the external forces on EEs respectively;
    - wrench_mapping.m = function used to map the external forces in the CDTS w.r.t to absolute and relative frame;
    - stiff_fixed.m, trivial_solution.m = script used to compute trivial task or with no stiffness adaptation. 
- **Functions**:
    - it contains all the utils functions used, e.g dynamics matrix robots and dual quaternion matrices to map into vector space. 
- **Vrep_utils**:
    - it contains the matlab scripts needed to interface with robot and api, and to retrieve joint positions and info from the robot framework.
  
Please note that all the files are written in Matlab. To run the simulations CoppeliaSim needs to be downloaded on local pc.
