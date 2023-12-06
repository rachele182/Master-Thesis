# Read Me
<font color="green">**Author**:</font>  Rachele Nebbia Colomba  
<font color="green">**Description of contents**:</font> 

In this folder you can find the code for variable addmittance controller for a bimanual manipulator using dual quaternion algebr.
The controller was designed by using ros_noetic version and FRANKA_ROS to interface with the robots.
Please refer to contents as follows:
1. panda controllers:
it contains all the files needed to run ros and interface with the two arms robot, specifically:  
**src**: contain .cpp files;  
**include**: header of cpp files with class definition and variables
**launch**: launch ros files for dual controller, admittance loop, and trajectory generatore  
**msg:** customed ros mgs for publisher and subscriber nodes  
2. rosbag: it contains the recorded data from different grasping tasks experiments and the script used to plot those data in matlab eg:  
**extrac_data.m**  
3. demoplots: it contains all the visual results of the threee different experiments carried out in terms of .svg images and .mov videos.

Please note that **CMake.txt** represents the configuration file used to determine the ros settings.
This need to be edited as well for local use. 