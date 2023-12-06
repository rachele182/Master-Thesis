## Adaptive Cooperative Admittance Controller

Step 1 ) Adapating relative stiffness:

Instructions to run the simulation of a grasping task: 

1) Open the corresponding CoppeliaSim scene inside the folder Adapting_stiffness:

   - dual_arm.ttt 

2) Run init.m file to include the parameters in the workspace.
   You can set the sampling time of the simulation as well as the time-frame by modyfying cdt and tfin respectively inside this file.
   
3) run grasping_task.m to start a new simulation of the task.

4) inside the folder Data you can find previous simulation outputs and the corresponding .m file to plot some results.
    
        More specifically:  
        run tight_grasp.m to visualize plots of trivial solutions for grasping task;
        run fixed_stiffness.m to visualize plots of CDAC controller without relative stiffness adaptation; 
        run adaptive_stiffness.m to visualize plots of CDAC controller without relative stiffness adaptation. 

To visualize the plots of the results of the three different simulation run performance_analysis.m

Step 2) Adapting both relative and absolute stiffness: 

Increase absolute stiffness (within stab conditions) during lifting phase to have better tracking performance of the nominal trajectory;

1) Same procedure of step 1: to start a new simulation of same task run grasping_task_absolute_adaptation.m;

2) Run the script rel_abs_adapter.m inside the data folder to see the comparison in terms of absolute pose tracking with and without absolute stiffness adaptation. 