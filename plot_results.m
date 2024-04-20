%%
close all;clear;clc;
%% load files
LMPC = load('Linear_MPC_data.mat');
NLMPC = load('NL_Poly_MPC_data.mat');
PID = load('PID_data.mat');
%% plot
t = 0:0.1:30;
y_ref = [20*ones(1,100) 25*ones(1,100) 20*ones(1,150)];
Vx_ref = 25;
Vy_ref = 0;

figure
subplot(3, 2, 1)
plot(t, PID.x(3, :),'LineWidth',1.5)
hold on
plot(t, LMPC.x(3, :),'LineWidth',1.5)
plot(t, NLMPC.x(3, :),'LineWidth',1.5)
plot([t(1) t(end)], [Vx_ref Vx_ref], ':b')
hold off
legend('PID', 'LMPC','NLMPC', 'reference')
xlabel('time [s]')
ylabel('Horizontal Velocity [m/s]')

subplot(3, 2, 2)
plot(t, PID.x(4, :),'LineWidth',1.5)
hold on
plot(t, LMPC.x(4, :),'LineWidth',1.5)
plot(t, NLMPC.x(4, :),'LineWidth',1.5)
hold off
legend('PID', 'LMPC','NLMPC')
xlabel('time [s]')
ylabel('Vertical Velocity [m/s]')

subplot(3, 2, 3)
plot(t, PID.x(2, :),'LineWidth',1.5)
hold on
plot(t, LMPC.x(2, :),'LineWidth',1.5)
plot(t, NLMPC.x(2, :),'LineWidth',1.5)
plot(t, y_ref(1:length(t)), ':b')
hold off
legend('PID', 'LMPC','NLMPC', 'reference')
xlabel('time [s]')
ylabel('Vertical Position [m]')

subplot(3, 2, 4)
plot(PID.x(1, :), PID.x(2, :),'LineWidth',1.5)
hold on
plot(LMPC.x(1, :), LMPC.x(2, :),'LineWidth',1.5)
plot(NLMPC.x(1, :), NLMPC.x(2, :),'LineWidth',1.5)
hold off
legend('PID', 'LMPC','NLMPC')
xlabel('Horizontal Position [m]')
ylabel('Vertical Position [m]')

subplot(3, 2, 5)
plot(t(1:end-1), PID.u(1, :),'LineWidth',1.5)
hold on
plot(t, LMPC.u(1, :),'LineWidth',1.5)
plot(t(1:end-1), NLMPC.u(1, :),'LineWidth',1.5)
hold off
legend('PID', 'LMPC','NLMPC')
xlabel('time [s]')
ylabel('Wing Flapping Frequency [Hz]')

subplot(3, 2, 6)
plot(t(1:end-1), PID.u(2, :),'LineWidth',1.5)
hold on
plot(t, LMPC.u(2, :),'LineWidth',1.5)
plot(t(1:end-1), NLMPC.u(2, :),'LineWidth',1.5)
hold off
legend('PID', 'LMPC','NLMPC')
xlabel('time [s]')
ylabel('Tail Wing Angle [deg]')