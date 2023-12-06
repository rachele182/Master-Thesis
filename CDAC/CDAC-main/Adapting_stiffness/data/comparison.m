%%cfr
mu = 0.25;
%% Adaptive stiffness plots
load("data/stiff.mat");
load("data/k_increase.mat"); 

load("data/k_fixed.mat"); 



f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,data.f1_ext(:,2),'LineWidth',2);
hold on, grid on
plot(tt,(data.f1_ext(:,3))*mu*-1,'r--','LineWidth',2.5);
hold on, grid on
plot(tt,(data2.f1_ext(:,2)),'LineWidth',2,'Color',[0.6 0.8 0.2]);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([1.8,4])
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('adaptive','desired friction','fixed', 'latex', 'FontSize', 10)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,2),'LineWidth',2);
hold on, grid on
plot(tt,(data.f2_ext(:,3))*mu,'r--','LineWidth',2.5);
hold on, grid on
plot(tt,(data2.f2_ext(:,2)),'LineWidth',2,'Color',[0.6 0.8 0.2]);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([1.8,4])
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)




%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
grid on
hold on
plot(tt,data2.pos1(3,:)*1.01,'LineWidth',2,'Color',[0.6 0.8 0.2]);
ylabel('$y_1/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([2.5 4])
ylim([-0.051 -0.046])
legend('adaptive','contact point','fixed','Interpreter', 'latex', 'FontSize', 10)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.pos2(3,:),'b','LineWidth',2);
hold on
grid on
plot(tt,0.05*ones(401,1),'r--','LineWidth',2.5)
hold on
grid on
plot(tt,data2.pos2(3,:),'b','LineWidth',2,'Color',[0.6 0.8 0.2]);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$y_2/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([2.5 4])
ylim([0.046 0.051])


