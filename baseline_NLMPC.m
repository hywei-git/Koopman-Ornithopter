clear; close all; clc;
%% Setting Up NLMPC
step_mpc = 1;                               % Step size of mpc
simtime = 20;                               % Simulation duration
N_step = simtime/step_mpc;                  % Number of steps
x0 = [0; 15; 10; 0; 10; 0];                 % Initial [x, y, Vx, Vy, delta, delta_dot]'
x = zeros(6,N_step+1);                      % Output state matrix
x(:,1) = x0;                                % Setup initial condition into x
u = zeros(2,N_step);                        % Output control sequence
t = (0:step_mpc:simtime)';                  % Simulation time vector                
N = 5;                                      % Horizon length
u0 = zeros(N*2,1);                          % Decision vector (freq; theta)
f0 = 2;                                     % First iter initial guess
theta0 = 0;                                 % First iter initial guess
for i = 1:N
    u0(2*i-1) = f0;
    u0(2*i) = theta0;
end

% NLMPC options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');       
options.MaxFunctionEvaluations = 5e6;
%% Constraints and References
y_min = 0;                                  % Elavation lower bound
y_ref = [20*ones(1,20) 25*ones(1,20) 20*ones(1,22)];    % Reference elavation vector
Vx_ref = 25;                                % Reference Vx
Vy_ref = 0;                                 % Reference Vy
lb = zeros(size(u0));                       % Input lower bound
ub = lb;                                    % Input upper bound
f_min = 0.1;                                % Min frequency
f_max = 5;                                  % Max frequency
theta_min = -10;                            % Min theta deg
theta_max = 10;                             % Max theta deg

% Stack up lower and upper bound vectors
for i = 1:N
    lb(2*i-1) = f_min;
    lb(2*i) = theta_min;
    ub(2*i-1) = f_max;
    ub(2*i) = theta_max;
end

% Input rate limiter
u_rate_max = 1*[5; 5];
%% Start Simulation
tic;
for iter = 1:N_step/N
    % SQP approximates optimal control sequence for the following N steps
    u_opt = fmincon(@(u)obj_fun(u, N, step_mpc, x(:,iter*N-N+1), Vx_ref, Vy_ref, y_ref(1, iter*N-N+1:iter*N)), u0, [], [] , [], [], lb, ub, [], options);
    
    % Initial guess at next iter is optimal input at previous iter
    u0 = u_opt;
                       
    for i = 1:N
        % Store optimal u
        u_next = u_opt(2*i-1:2*i);
        u(:,iter*N-N+i) = u_next;
        % Simulate the system 1 step forward using optimal control sequence
        [~,x_temp] = ode45(@(t,x) original_state_fcn(x,u_next) , [0 step_mpc], x(3:end,iter*N-N+i));
        % Append x,y position by manual integration
        x_temp = [x(1:2,iter*N-N+i)+step_mpc*x_temp(end,1:2)'; x_temp(end,:)'];
        % Store simulated state at next step
        x(:,iter*N-N+i+1) = x_temp;
    end
end
toc;

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
plot(t, y_ref(1:length(t)), ':r');
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


%% Function Definition
% Objective function for NLMPC
function J = obj_fun(u, N, step_mpc, x0, Vx_ref, Vy_ref, y_ref)
    % Initialize output state matrix
    x = zeros(length(x0),N+1);
    x(:,1) = x0;
    for i = 1:N
        % Simulate using simplified polynomial function
        [~,x_temp] = ode15s(@(t,x) original_state_fcn(x,u(2*i-1:2*i)) , [0 step_mpc], x(3:end,i));
        x_temp = [x(1:2,i)+step_mpc*x_temp(end,1:2)'; x_temp(end,:)'];  
        % Store simulated states
        x(:,i+1) = x_temp';
    end
    % Cost = Track Vx, y
    J = sum((x(3,2:end) - Vx_ref).^2)+0.1*sum((x(2,2:end) - y_ref).^2);
end

% Nonlinear constraints
function [g, h] = nl_constraint(u, N, step_mpc, x0, y_min)
    % Initialize output state matrix
    x = zeros(length(x0),N+1);
    x(:,1) = x0;
    for i = 1:N
        % Simulate using simplified polynomial function
        [~,x_temp] = ode45(@(t,x) nonlinear_simplified_state_fcn(x, u(2*i-1:2*i)) , [0 step_mpc], x(:,i));
        % Store simulated states
        x(:,i+1) = x_temp(end,:)';
    end
    
    % Inequality constraint g < 0
    g = y_min*ones(N,1) - x(2,2:end);
    % Equality constraint h = 0
    h = 0;
end