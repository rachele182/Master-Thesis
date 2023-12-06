%% Adaptive stiffness plots
load("data/stiff.mat");
load("data/k_increase.mat"); 


%% Internal stresses 
mu = 0.2; %friction coefficient 


f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
title('Internal stresses')
grid on
hold on
plot(tt,data.f1_ext(:,2),'LineWidth',2);
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,2),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


%% gravity
f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
title('Weight forces')
grid on
hold on
plot(tt,data.f1_ext(:,3),'LineWidth',2);
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,3),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

%% Internal stresses comparison with desired friction

f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,data.f1_ext(:,2),'LineWidth',2);
hold on, grid on
plot(tt,(data.f1_ext(:,3))*mu*-1,'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([2,4])
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,2),'LineWidth',2);
hold on, grid on
plot(tt,(data.f2_ext(:,3))*mu,'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([2,4])
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


%% Relative pose
f1 = figure;
f1.Renderer = 'painters';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos_r_d(2,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_r(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos2_c(2,:),'b','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 2)
grid on
hold on
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos2_c(3,:),'b','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(3, 1, 3)
grid on
hold on
plot(tt,data.pos_r_d(4,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_r(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos2_c(4,:),'b','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','curr','comp','Interpreter', 'latex', 'FontSize', 10)


%% Absolute pose
f1 = figure;
f1.Renderer = 'painters';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos_abs_d(2,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_a(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos1_c(2,:),'b','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 2)
grid on
hold on
plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos1_c(3,:),'b','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(3, 1, 3)
grid on
hold on
plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos1_c(4,:),'b','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','curr','comp','Interpreter', 'latex', 'FontSize', 10)



%% Adaptive stiffnes values 
f1 = figure;
f1.Renderer = 'painters';

grid on
hold on
plot(tt,kz_data,'b','LineWidth',2); 
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$kz/\mathrm{N/m}$', 'Interpreter', 'latex', 'FontSize', 12)

%% EEs position

f1 = figure;
f1.Renderer = 'painters';

subplot(2, 1, 1)
grid on
hold on
plot(tt,data.pos1(3,:),'b','LineWidth',2); 
hold on
grid on
plot(tt,-0.05*ones(401,1),'r--','LineWidth',2.5)
ylabel('$y1/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('pos','contact point','Interpreter', 'latex', 'FontSize', 10)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.pos2(3,:),'b','LineWidth',2);
hold on
grid on
plot(tt,0.05*ones(401,1),'r--','LineWidth',2.5)
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$y2/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)



