%% Adaptive absolute stiffness plots comparison
load("data/stiff.mat");
load("data/k_increase.mat"); 
load("Adapting_stiffness/data/abs_rel_adapter.mat");
load("Adapting_stiffness/data/k_abs_var.mat"); 

%% Absolute pose comparison
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
plot(tt,data_abs.pos_a(2,:),'b','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 2)
grid on
hold on
plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data_abs.pos_a(3,:),'b','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(3, 1, 3)
grid on
hold on
plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',2.5); 
hold on, grid on
plot(tt,data.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data_abs.pos_a(4,:),'b','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','fixed','var','Interpreter', 'latex', 'FontSize', 10)



%% Adaptive relative stiffness values 
f1 = figure;
f1.Renderer = 'painters';

grid on
hold on
plot(tt,kz_data,'b','LineWidth',2); 
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$kz/\mathrm{N/m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('Relative stiffness')

%% Adaptive absolute stiffness values 
f1 = figure;
f1.Renderer = 'painters';

grid on
hold on
plot(tt,kz1_data,'b','LineWidth',2); 
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$kz/\mathrm{N/m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('Absolute stiffness')

