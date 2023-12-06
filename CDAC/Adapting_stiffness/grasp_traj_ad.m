function [xad,dxad,ddxad,xrd,dxrd,ddxrd,grasp_data,phase_data] = grasp_traj_ad(x1_in,x2_in,time)

%% Description: generates minimum jerk task-space trajectory (DQ representation)

%%Inputs:  x1_in = initial EE1 pose; [8x1]
%          x2_in = initial EE2 pose; [8x1]
%          time = simulation time.
%%Outputs: xad,dxad,ddxad = desired absolute trajectory; [8x1]
%          xrd,dxrd,ddxrd = desired relative trajectory; [8x1]
%          phase_data = flag acknowleding phases of grasping task; (free-motion,contact,lift)
%          grasp_data = flag acknowleding lifting phase; (for external forces model)  

cdt = time(2)-time(1); %sampling time

%%Initialize
%absolute pose trajectory
xad = [zeros(size(time,2),8)];
dxad = [zeros(size(time,2),8)];
ddxad = [zeros(size(time,2),8)];

%relative pose trajectory
xrd = [zeros(size(time,2),8)];
dxrd = [zeros(size(time,2),8)];
ddxrd = [zeros(size(time,2),8)];

grasp_data = [zeros(size(time,2),1)]; 
phase_data = [zeros(size(time,2),1)];

grasp = 0; 
phase = 1; 

%%retrieve initial conditions
%left arm
p01 = vec4(x1_in.translation);
r01 = vec4(P(x1_in));
pos_i = [p01(2);p01(3);p01(4)];

%right arm
p02 = vec4(x2_in.translation);
r02 = vec4(P(x2_in));
pos_i_2= [p02(2);p02(3);p02(4)];

i = 1;  

for i = 1:size(time,2)
    if (time(i) >=0 && time(i) < 0.5) %go down
        pos_f = pos_i + [0;0;-0.15];
        pos_2_f = pos_i_2 + [0;0;-0.15]; 
        tf = 0.5;
        t = time(i);
    elseif (time(i) >=0.5 && time(i) < 0.7) %pause
        pos_i = [p01(2);p01(3);p01(4)-0.15];
        pos_i_2 = [p02(2);p02(3);p02(4)-0.15];
        pos_f = pos_i;
        pos_2_f = pos_i_2;
        tf = 0.2;
        t = time(i) - 0.5;
    elseif (time(i) >= 0.7 && time(i) < 1.7) %approach obj
        pos_i = [p01(2);p01(3);p01(4)-0.15];
        pos_i_2 = [p02(2);p02(3);p02(4)-0.15];
        pos_f = pos_i + [0;0.2478;-0.05]; 
        pos_2_f = pos_i_2 + [0;-0.2478;-0.055]; 
        tf = 1;
        t = time(i) - 0.7;
        phase = 2; 
    elseif (time(i) >= 1.7 && time(i) < 2) %pause
        pos_i = [p01(2);p01(3)+0.2478;p01(4)-0.15-0.05];
        pos_i_2 = [p02(2);p02(3)-0.2478;p02(4)-0.15-0.055];
        pos_f = pos_i; 
        pos_2_f = pos_i_2; 
        tf = 0.3;
        t = time(i) - 1.7;
        grasp = 1;
        phase = 3; 
    elseif (time(i) >= 2 && time(i) < 2.4) %go up
        pos_i = [p01(2);p01(3)+0.2478;p01(4)-0.15-0.05];
        pos_i_2 = [p02(2);p02(3)-0.2478;p02(4)-0.15-0.055];
        pos_f = pos_i + [0;0;0.15]; 
        pos_2_f = pos_i_2 + [0;0;0.15]; 
        tf = 0.4;
        t = time(i) - 2;
        grasp = 1;
        phase = 3; 
    else
        pos_i = [p01(2);p01(3)+0.2478;p01(4)-0.05];
        pos_i_2 = [p02(2);p02(3)-0.2478;p02(4)-0.055];
        pos_f = pos_i;
        pos_2_f = pos_i_2; 
        tf = 1000;
        t = time(i) - 2.4;
        grasp = 1; 
        phase = 3; 
    end

    %% Minimum jerk interpolation
    %arm1
    zd = pos_i + (pos_i - pos_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dzd = (pos_i - pos_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddzd = (pos_i - pos_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));
    %arm2
    z2_d = pos_i_2 + (pos_i_2 - pos_2_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dz2_d = (pos_i_2 - pos_2_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddz2_d = (pos_i_2 - pos_2_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));
    
    %% Compute trajectory (DQ)
    %arm1
    x1_des = vec8(DQ(r01) + 0.5*DQ.E*(DQ(zd)*DQ(r01)));
    dx1_des =  vec8(0.5*DQ.E*(DQ(dzd)*DQ(r01)));
    ddx1_des = vec8(0.5*DQ.E*(DQ(ddzd)*DQ(r01)));
 
    %arm2
    x2_des = vec8(DQ(r02) + 0.5*DQ.E*(DQ(z2_d)*DQ(r02)));
    dx2_des =  vec8(0.5*DQ.E*(DQ(dz2_d)*DQ(r02)));
    ddx2_des = vec8(0.5*DQ.E*(DQ(ddz2_d)*DQ(r02)));
    
    %Compute desired absolute and relative pose
    xr_des = vec8(DQ(x2_des)'*DQ(x1_des));
    dxr_des = vec8(DQ(dx2_des)'*DQ(x1_des) + DQ(x2_des)'*DQ(dx1_des));
    ddxr_des = vec8(DQ(ddx2_des)'*DQ(x1_des) + 2*DQ(dx2_des)'*DQ(dx1_des) + DQ(x2_des)'*DQ(ddx1_des)); 
    
    xa_des = vec8(DQ(x2_des)*(DQ(xr_des))^0.5);

    if(i == 1)
        dxa_des = zeros(8,1);
        ddxa_des = zeros(8,1);
    else
        dxa_des = (xa_des - xad(i-1,:)')/cdt; 
        ddxa_des = (dxa_des - dxad(i-1,:)')/cdt;
    end
   
    %Store outputs
    xad(i,:) = xa_des;
    dxad(i,:) = dxa_des;
    ddxad(i,:) = ddxa_des;
  
    xrd(i,:) = xr_des;
    dxrd(i,:) = dxr_des;
    ddxrd(i,:) = ddxr_des;

    grasp_data(i,:) = grasp; 
    phase_data(i,:) = phase; 
    
    i = i+1;
end

end
