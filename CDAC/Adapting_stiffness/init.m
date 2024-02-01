%    begin                : January 2022
%    authors              : Rachele Nebbia Colomba
%    copyright            : (C) 2022 Technical University of Munich // Universitä di pisa    
%    email                : rachelenebbia <at> gmail <dot> com

%%Description: Init file for grasping task simulation of bimanual system
%Here you can find all the parameters set (and editable)

%% Load data file
%% Clear workspace
clear;
close all;
clc;

%% Addapth;
p1 = genpath('Vrep_utils');
p2 = genpath('functions');
p3 = genpath('Adapting_stiffness');
p4 = genpath('data');
addpath(p1,p2,p3,p4); 

disp('Loading parameters..')

%% Simulation time Parameters
cdt = 0.01; %[s] sampling time
tfin = 4; %s 
time = 0:cdt:tfin; %simulation time
tt = time; 

%% Panda Joints limits 
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];
qc = 0.5*(q_min+q_max)'; 

%% Impedance matrices
%absolute impedance
mass = 1.5;  % [kg] 
I = eye(6);
Md1 = 1.5*I; %desired mass [kg]
Md2 = Md1; 
Kd1 = 300*I; %desired stiffness [N/m]
Bd1 = sqrt(4*Kd1*Md1); %desired damping [Ns/m] 

%%Default values for rotational stiffness/damping
k_def = 300;
d_def = 2*sqrt(k_def*1.5); %critically damped

%%Initial values for translational stiffness grasping/task 
%kt_def = 100; % [N/m]

%% Motion controller gains
kp = 300;
kd = 30;
ki = 100;

%% Environment parameters
g = 9.81; % [m/s^2] %gravity 
mass_obj = 0.5; % [kg]
weight = mass_obj*g; % [N] %object weight
width = 0.1; %[m] object dimension
k_table = 5000; %N/m, environment stiffness
k_obj = 500; %N/m, object stiffness

%friction coefficient
mu_nom = 0.4;
alpha = 0.25; %uncertainty of 25%
mu_wc = mu_nom*(1-alpha); %worst case 


disp('Done!')