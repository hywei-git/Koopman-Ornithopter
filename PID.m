clear; clc; close all;
%%
computation_time = 0;                       % Initialize computation time
simtime = 30;                               % Simulation duration
step = 0.1;                                 % Step size of mpc
x0 = [0; 15; 15; 0; 10; 0];                 % Initial [x, y, Vx, Vy, delta, delta_dot]'
x = zeros(6,simtime/step + 1);              % State matrix
x(:,1) = x0;                                % Add to state matrix
u = zeros(2,simtime/step);                  % Input matrix
t = 0:step:simtime;                         % Simulation time vector
umax = [5; 10];                             % Maximum input
umin = [0.1; -10];                          % Minimum input

% Parameters for [Vx, y]
kp = [4 0; 
      0 4];                                 % Proportional gain for Vx y tracking
kd = [0 0;
      0 12];                                % Derivative gain for Vx y tracking
ki = [0.4 0;
      0 0.2];                               % Integral gain for Vx y tracking
Vx_ref = 25;                                % Reference Vx
Vy_ref = 0;                                 % Reference Vy
y_ref = [20*ones(1,100) 25*ones(1,100) 20*ones(1,120)]; % Reference y
%y_ref = 15 - 5*cosd(2*(0:620));
%% Simulation
% initialize error
err_integral = [0; 0];
err_integral_max = [50; 50];
err_last = [0; 0];
for iter = 1:simtime/step
    tStart = tic;
    % Error [Vx, y]
    err = [Vx_ref - x(3,iter); y_ref(iter) - x(2,iter)];
    % Error integration
    err_integral = err_integral + err;
    % Integrater unwind
    err_integral = min(err_integral_max, max(-err_integral_max, err_integral));
    % combine kp, ki, kd
    u(:,iter) = kp*err + kd*(err - err_last) + ki*err_integral;
    % Saturate PID output
    u(:,iter) = min(umax, max(umin, u(:,iter)));
    tEnd = toc(tStart);
    % update computation time
    computation_time = computation_time + tEnd;
    
    % Simulate
    [~,x_temp] = ode45(@(t,x) original_state_fcn(x,u(:,iter)) , [0 step], x(3:end,iter));
    % Save new state
    x(:,iter+1) = [x(1:2,iter)+step*x_temp(end,1:2)'; x_temp(end,:)'];
    % Update last error values
    err_last = err;
end
%% Plot
fig1 = figure(1);
fig1.WindowState = 'maximized';
subplot(2,2,1);
plot(t,x(3,:),'-r','LineWidth',1.5);
hold on;
plot([0 t(end)], [Vx_ref, Vx_ref], ':r');
plot(t,x(4,:),'-b','LineWidth',1.5);
plot([0 t(end)], [Vy_ref, Vy_ref], ':b');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend("V_x", "V_x Ref", "V_y", "V_y Ref");
subplot(2,2,2);
plot(t,x(2,:),'-r','LineWidth',1.5);
hold on;
plot(t(2:end), y_ref(1:simtime/step), ':r');
xlabel('Time [s]');
ylabel('Vertical Position [m]');
legend("y", "y Ref");
subplot(2,2,3);
plot(t(2:end),u(1,:),'-r','LineWidth',1.5);
xlabel('Time [s]');
ylabel('Frequency [Hz]');
subplot(2,2,4);
plot(t(2:end),u(2,:),'-r','LineWidth',1.5);
xlabel('Time [s]');
ylabel('Theta [deg]');

%% save to file
save("PID_data.mat", 'x', 'u', 'computation_time');