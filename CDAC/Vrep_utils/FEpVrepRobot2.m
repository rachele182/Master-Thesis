%***************************************************************************
%                           FEpVrepRobot2.m -  description
%                           ----------------------------
%    begin                : July 2020
%    authors              : Riddhiman Laha, Luis F.C. Figueredo
%    copyright            : (C) 2021 Technical University of Munich
%    email                : riddhiman <dot> laha <at> tum <dot> de
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% **************************************************************************
% Purpose
% --------------------------------------------------------------------------
% Class for defining FE Panda Robot (right)
% 
% **************************************************************************
classdef FEpVrepRobot2 < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
    end
    
    methods 
        function obj = FEpVrepRobot2(robot_name,vrep_interface)
            
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'Franka2')
                error('Franka2')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:7
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end            
   
        function send_q_to_vrep(obj,q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names,65536);
        end
        
        function kin = kinematics(obj)
            %Standard D-H of FE Panda
            FEp_DH_theta  = [0,     0,      0,      0,        0,        0,      0];
            FEp_DH_d      = [0.333, 0,      0.316,  0,        0.384,    0,      0.107];
            FEp_DH_a      = [0,     0,      0.0825, -0.0825,  0,        0.088   0.0003];
            FEp_DH_alpha  = [-pi/2, pi/2,   pi/2,   -pi/2,    pi/2,     pi/2    0];

            FEp_DH_matrix = [FEp_DH_theta;
                            FEp_DH_d;
                            FEp_DH_a;
                            FEp_DH_alpha];

            kin = DQ_SerialManipulator(FEp_DH_matrix,'standard');
            % We set the transformation from the world frame to the robot
            % base frame. Therefore, the end-effector pose is given by
            % pose_effector = transformation_from_world_to_base*fkm(q);
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
%             kin.set_effector(1+0.5*DQ.E*DQ.k*0.1070);
        end
    end
end