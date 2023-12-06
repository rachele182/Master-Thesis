function [K,D] = abs_stiff_adapter(time_curr,time_prec,pos,e_pos,F_ext,phase)

%% Description: Modulate stiffness and damping on each cartesian dof of the impedance controller to guarantee good
%%              tracking perfomance and safe interaction.

%%Inputs:     pos = current position [1x1]
%             e_pos = position erro [1x1]
%             F_ext = external force on EE; [6x1]
%             phase = flag acknowledging the current desired trajectory phase
%%Outputs:    K = stiffness matrix [6x6]
%             D = damping matrix [6x6]

%%Parameters: F_max = treshold of disturbance
%             F_int_max = maximum interaction force
%             k_default = default value of stiffness
%             k_max = maximum stiffness value
%             a0,berta,csi = parameters from stability conditions
%             mass = apparent mass
%             z_table = environment contact position

berta = 0.98;
csi = 1;
a0 = 0.99;
F_max = 1; %N
F_int_max = 5; %N
mass = 1.5; %kg
k_max = 1000; %N/m
k_default = 300; %N/m
d_default = sqrt(4*mass*k_default);
kr_rot = 300;
dr_rot = 2*sqrt(mass*kr_rot);
z_table = 0.2942; %contact position

%% Initialization
persistent ki int

% persistent initialization
if isempty(ki)
    ki = k_default;
end

if isempty(int)
    int = 1000;
end


%% Compute k
if (phase == 1 || phase ==2)
    if abs(F_ext) > F_max %contact is detected
        ki = 0.995*ki; %arbitrarly decrease k
        if int == 1000 %contact position not set yet
            int = pos;
        end
        % if interaction force is higher than threshold
        if ki*abs(e_pos) > F_int_max
            % set k
            ktemp_x = F_int_max/abs(e_pos);
            if ktemp_x < ki
                ki = ktemp_x;
            end
        end
    end
else
    if pos > z_table  %safe to increase k
        k_dot = berta*(4*a0*sqrt(ki/mass)*(ki)^(3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki)); %maximum variation within stability conditions
        k_tempx = ki + k_dot*(time_curr -time_prec);
        ki = k_tempx;
        if k_tempx > k_max
            ki = k_max;
        end
    end
end

k = ki;
d = sqrt(4*mass*k);

ka_t = [k;k;k];
da_t = [d;d;d];

K = diag([kr_rot*ones(3,1);ka_t]);
D = diag([dr_rot*ones(3,1);da_t]);


end

