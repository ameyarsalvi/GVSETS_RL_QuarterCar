

%%
close all
X = squeeze(stateData);
U = squeeze(actionData);
timeData = 0.1*(1:length(timeData));

FS = 16;

% x dot plot
f1 = figure('color','w');
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;
hold on 
plot(timeData,X(8,:),'-b','linewidth',1.2)
xlim([0,100])
ylim([0,30])
plot(timeData(1:1000),Ter,'--k')
box on 
% plot(xlim,[10.5,10.5],'--k')
xlabel('Time ($t$) [s]','Interpreter','latex')
ylabel('Longitudinal Velocity ($\dot{x}$) [m/s]','Interpreter','latex')

% y dot plot
f2 = figure('color','w');
plot(timeData,X(10,:),'-b','linewidth',1.2)
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;
xlim([0,100])
xlabel('Time ($t$) [s]','Interpreter','latex')
ylabel('Vertical Velocity ($\dot{y}$) [m/s]','Interpreter','latex')

% Torque plot
f3 = figure('color','w');
plot(timeData(1:end-1),U(1,:),'r','linewidth',1.2)
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;
xlabel('Time ($t$) [s]','Interpreter','latex')
ylabel('Torque ($T$) [N$\cdot$m]','Interpreter','latex')
xlim([0,100])

% Stiffness plot
f4 = figure('color','w');
plot(timeData(1:end-1),15000+U(2,:),'r','linewidth',1.2)
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;
xlabel('Time ($t$) [s]','Interpreter','latex')
ylabel('Stiffness ($k$) [N/m]','Interpreter','latex')
xlim([0,100])
ylim([0 2.5e4])

print(f1,'VelPlot_2vel','-depsc')
print(f2,'OscPlot_2vel','-depsc')
print(f3,'TorquePlot_2vel','-depsc')
print(f4,'StiffnessPlot_2vel','-depsc')