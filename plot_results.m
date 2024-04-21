%%
close all;clear;clc;
%% load files
LMPC = load('Linear_MPC_data.mat');
LMPC_OR = load('Linear_MPC_online_regression_data.mat');
LMPC_OR_IC = load('Linear_MPC_online_regression_integral_feedback_data.mat');
NLMPC = load('NL_Poly_MPC_data.mat');
PID = load('PID_data.mat');
%% plot
% references and time
t = 0:0.1:30;
y_ref = [20*ones(1,100) 25*ones(1,100) 20*ones(1,150)];
Vx_ref = 25;
Vy_ref = 0;

% plot states of three different controllers
figure
subplot(2, 1, 1)
plot(t, PID.x(3, :),'LineWidth',1.5)
hold on
plot(t, LMPC.x(3, :),'LineWidth',1.5)
plot(t, NLMPC.x(3, :),'LineWidth',1.5)
plot([t(1) t(end)], [Vx_ref Vx_ref], ':b')
hold off
legend('PID', 'LMPC','NLMPC', 'reference', "Location", "southeast")
xlabel('time [s]')
ylabel('Horizontal Velocity [m/s]')

subplot(2, 1, 2)
plot(t, PID.x(2, :),'LineWidth',1.5)
hold on
plot(t, LMPC.x(2, :),'LineWidth',1.5)
plot(t, NLMPC.x(2, :),'LineWidth',1.5)
plot(t, y_ref(1:length(t)), ':b')
hold off
legend('PID', 'LMPC','NLMPC', 'reference')
xlabel('time [s]')
ylabel('Vertical Position [m]')

% plot inputs of three different controllers
figure
subplot(2, 1, 1)
plot(t(1:end-1), PID.u(1, :),'LineWidth',1.5)
hold on
plot(t, LMPC.u(1, :),'LineWidth',1.5)
plot(t(1:end-1), NLMPC.u(1, :),'LineWidth',1.5)
hold off
legend('PID', 'LMPC','NLMPC')
xlabel('time [s]')
ylabel('Wing Flapping Frequency [Hz]')

subplot(2, 1, 2)
plot(t(1:end-1), PID.u(2, :),'LineWidth',1.5)
hold on
plot(t, LMPC.u(2, :),'LineWidth',1.5)
plot(t(1:end-1), NLMPC.u(2, :),'LineWidth',1.5)
hold off
legend('PID', 'LMPC','NLMPC')
xlabel('time [s]')
ylabel('Tail Wing Angle [deg]')

% plot states of LMPC, LMPC+online regression, LMPC+online regression+integral feedback
figure
subplot(2, 1, 1)
plot(t, LMPC.x(3, :),'LineWidth',1.5)
hold on
plot(t, LMPC_OR.x(3, :),'LineWidth',1.5)
plot(t, LMPC_OR_IC.x(3, :),'LineWidth',1.5)
plot([t(1) t(end)], [Vx_ref Vx_ref], ':b')
hold off
legend('LMPC', 'LMPC with online regression','LMPC with online regression and intergral feedback', 'reference', "Location", "southeast")
xlabel('time [s]')
ylabel('Horizontal Velocity [m/s]')

subplot(2, 1, 2)
plot(t, LMPC.x(2, :),'LineWidth',1.5)
hold on
plot(t, LMPC_OR.x(2, :),'LineWidth',1.5)
plot(t, LMPC_OR_IC.x(2, :),'LineWidth',1.5)
plot(t, y_ref(1:length(t)), ':b')
hold off
legend('LMPC', 'LMPC with online regression','LMPC with online regression and integral feedback', 'reference', "Location", "southeast")
xlabel('time [s]')
ylabel('Vertical Position [m]')

% plot inputs of LMPC, LMPC+online regression, LMPC+online regression+integral feedback
figure
subplot(2, 1, 1)
plot(t, LMPC.u(1, :),'LineWidth',1.5)
hold on
plot(t, LMPC_OR.u(1, :),'LineWidth',1.5)
plot(t, LMPC_OR_IC.u(1, :),'LineWidth',1.5)
hold off
legend('LMPC', 'LMPC with online regression','LMPC with online regression and intergral feedback')
xlabel('time [s]')
ylabel('Wing Flapping Frequency [Hz]')

subplot(2, 1, 2)
plot(t, LMPC.u(2, :),'LineWidth',1.5)
hold on
plot(t, LMPC_OR.u(2, :),'LineWidth',1.5)
plot(t, LMPC_OR_IC.u(2, :),'LineWidth',1.5)
hold off
legend('LMPC', 'LMPC with online regression','LMPC with online regression and integral feedback')
xlabel('time [s]')
ylabel('Tail Wing Angle [deg]')

computation_time = ["PID","LMPC","NLMPC","LMPC-OR","LMPC-OR-IC"]';
computation_time(1,2) = PID.computation_time*1000/300;
computation_time(2,2) = LMPC.computation_time*1000/300;
computation_time(3,2) = NLMPC.computation_time*1000/300;
computation_time(4,2) = LMPC_OR.computation_time*1000/300;
computation_time(5,2) = LMPC_OR_IC.computation_time*1000/300;
computation_time