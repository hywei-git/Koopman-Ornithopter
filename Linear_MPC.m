clear; close all; clc;
%% User Input
% Toggle on/off online regression
online_regression_on = true;
% Toggle on/off integral feedback
integral_feedback_on = false;

%% Set Up Controller Parameters
computation_time = 0;   % Computational time counter
N = 5;                  % Horizon length
T = 0.1;                % MPC step size
t_end = 30;             % Simulation length
t = (0:T:t_end)';       % Simulation time vector
dim_state = 15;         % Dimension of state
dim_input = 2;          % Dimension of input
x_length = dim_state*N; % Length of stacked state vector
u_length = dim_input*N; % Length decision variable vector
k_i = 0.4;              % Integral gain

%% Setting Up Optimization Problem
load("linear_model.mat");                                   % Load linear koopman model
Ad = A;                                                     % Discretized A
Bd = B;                                                     % Discretized B
Vx_ref = 25;                                                % Reference Vx
Vy_ref = 0;                                                 % Reference Vy
y_ref = [20*ones(1,100) 25*ones(1,100) 20*ones(1,150)];     % Reference y trajectory
% y_ref = 15 - 5*cosd(2*(0:650));
w_vx = 1;                                                   % Penalty weight for vx
w_vy = 10;                                                  % Penalty weight for vy
w_y = 100;                                                  % Penalty weight for y
Q = zeros(dim_state);                                       % Original weight matrix on x^2
Q(3,3) = w_vx;                                              % Penalizes Vx
Q(2,2) = w_y;                                               % Penalizes y
%Q(4,4) = w_vy;                                             % Penalizes Vy
R = zeros(dim_input);                                       % Original weight vector on u^2
R(2, 2) = 1;                                                % Penalizes large theta input
P = zeros(1,dim_state);                                     % Matrix that contains state references
P(3) = -2*w_vx*Vx_ref;
P(2) = -2*w_y*y_ref(1);
[x0, x0_og] = koopman_space_conversion(0,15,15,0,10,0);     % Convert (x,y, vx, vy, delta, delta_dot) to koopman space
u0 = ones(u_length,1);                                      % Initial guess for input

%% Setting Up QP (Direct Shooting)
% See report for the derivations
H_AB = zeros(x_length, u_length);
for i = 1:N
    for j = 1:i
        H_AB(i*dim_state-dim_state+1:i*dim_state, j*dim_input-dim_input+1:j*dim_input) = Ad^(i-j)*Bd;
    end
end
Q_bar = zeros(u_length);
r_vec = zeros(1, u_length);
for i = 0:N-1
    Q_bar = Q_bar + H_AB(i*dim_state+1:i*dim_state+dim_state,:)'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
    r_vec = r_vec + 2*(x0'*(Ad^(i+1))'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:)) + P*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
    Q_bar(i*dim_input+1:i*dim_input+dim_input,i*dim_input+1:i*dim_input+dim_input) = Q_bar...
         (i*dim_input+1:i*dim_input+dim_input,i*dim_input+1:i*dim_input+dim_input) + R;
end

%% Setting up input constraints
u_max = [5; 10];                              % Maximum for [f, theta]
u_min = [0.1; -10];                           % Minimum for [f, theta]
u_min_vec = zeros(u_length,1);                
u_max_vec = u_min_vec;

% Stack up the min/max values for the entire control sequence
for i = 0:N-1
    u_min_vec(i*dim_input+1:i*dim_input+dim_input) = u_min;
    u_max_vec(i*dim_input+1:i*dim_input+dim_input) = u_max;
end

% Aineq*u - bineq <= 0
Aineq = [-eye(u_length); eye(u_length)];
bineq = [-u_min_vec; u_max_vec];

%% Start Simulation
integral_error = 0;                 % Accumulated error
u_next = [0.1; 0];                  % Initialize input vector
x = zeros(dim_state, t_end/T + 1);  % Initialize koopman state matrix
x_og = zeros(6, t_end/T + 1);       % Initialize state matrix
x(:,1) = x0;
x_og(:,1) = x0_og;
u = u_next;
tic;
for iter = 1:(t_end/T)
    tStart = tic;

    % Updating r_vec and constraints before next step
    r_vec = zeros(1, u_length);
    for i = 0:N-1
        % First, update P(2) since y_ref is function of time
        P(2) = -2*w_y*y_ref(iter);
        r_vec = r_vec + 2*(x(:,iter)'*(Ad^(i+1))'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:)) + P*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
    end

    % LMPC caculates next input to system
    u_next = NextInputLMPC(Q_bar, r_vec, Aineq, bineq, u0, dim_input);

    % Apply integral control and saturate limits
    u_next = u_next+integral_feedback_on*0.4*[integral_error;0];
    u_next = min([5;10], max([0.1;-10], u_next));

    % Update computational time
    tEnd = toc(tStart);
    computation_time = computation_time + tEnd;

    % Simulate system one step forward when apply optimal input
    [t_temp,x_og_temp] = ode45(@(t,x) original_state_fcn(x,u_next) , [0 T], x_og(3:end,iter));

    % Integrate error and saturate to avoid winding up
    integral_error = integral_error + Vx_ref - x_og_temp(end,1);
    integral_error = min(3, max(-6, integral_error));

    % Simulate x,y position using forward euler
    x_og(1:2,iter+1) = x_og(1:2,iter) + T*x_og(3:4,iter);
    x_og(3:end,iter+1) = x_og_temp(end,:)';

    % Convert the new states into koopman space for next optimization
    % iteration
    [x(:,iter+1), ~] = koopman_space_conversion(x_og(1,iter+1),x_og(2,iter+1),x_og(3,iter+1),...
                                          x_og(4,iter+1),x_og(5,iter+1),x_og(6,iter+1));
    % Store input vector
    u = [u u_next];  
    
    % Online regression
    % Updates once 75 steps 
    if rem(iter, 75) == 0 && online_regression_on
        % Update A,B matrices using past flight data
        [updated_A, updated_B] = online_regression(Ad, Bd, x(:, 1:iter-1), u(:, 1:iter-1));

        % Linear blending
        Ad = updated_A*0.1 + Ad*0.9;
        Bd = updated_B*0.1 + Bd*0.9;

        % Formulate QP problem again using new A,B matrices
        H_AB = zeros(x_length, u_length);
        for i = 1:N
            for j = 1:i
                H_AB(i*dim_state-dim_state+1:i*dim_state, j*dim_input-dim_input+1:j*dim_input) = Ad^(i-j)*Bd;
            end
        end
        Q_bar = zeros(u_length);
        r_vec = zeros(1, u_length);
        for i = 0:N-1
            Q_bar = Q_bar + H_AB(i*dim_state+1:i*dim_state+dim_state,:)'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
            Q_bar(i*dim_input+1:i*dim_input+dim_input,i*dim_input+1:i*dim_input+dim_input) = Q_bar...
                (i*dim_input+1:i*dim_input+dim_input,i*dim_input+1:i*dim_input+dim_input) + R;
        end

    end
end
toc;

%% Plot
fig1 = figure(1);
fig1.WindowState = 'maximized';
subplot(2,2,1);
plot(t,x_og(3,:),'-r','LineWidth',1.5);
hold on;
plot([0 t(end)], [Vx_ref, Vx_ref], ':r');
plot(t,x_og(4,:),'-b','LineWidth',1.5);
plot([0 t(end)], [Vy_ref, Vy_ref], ':b');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend("V_x", "V_x Ref", "V_y", "V_y Ref");
subplot(2,2,2);
plot(t,x_og(2,:),'-r','LineWidth',1.5);
hold on;
plot(t(2:end), y_ref(1:t_end/T)', ':r');
xlabel('Time [s]');
ylabel('Vertical Position [m]');
legend("y", "y Ref");
subplot(2,2,3);
plot(t(2:end),u(1,2:end)','-r','LineWidth',1.5);
xlabel('Time [s]');
ylabel('Frequency [Hz]');
subplot(2,2,4);
plot(t(2:end),u(2,2:end),'-r','LineWidth',1.5);
xlabel('Time [s]');
ylabel('Theta [deg]');

%% Save to Files
if online_regression_on && integral_feedback_on
    save("Linear_MPC_online_regression_integral_feedback_data.mat", 'x', 'u', 'computation_time');
elseif online_regression_on && ~integral_feedback_on
    save("Linear_MPC_online_regression_data.mat", 'x', 'u', 'computation_time');
else
    save("Linear_MPC_data.mat", 'x', 'u', 'computation_time');
end
%% Function Definition
% This function calculates the optimal sequence using quadprog
function u_next = NextInputLMPC(Q_bar, r_vec, Aineq, bineq, u0, dim_input)
    options = optimoptions('quadprog', 'Algorithm', 'active-set','ObjectiveLimit',-1e20);
    [u_con, J_con] = quadprog(2*Q_bar, r_vec, Aineq, bineq, [], [], [], [], u0, options);
    % Only take the first entry in control sequence
    u_next = u_con(1:dim_input);
end

% This function converts given variables into state vector and koopman
% state vector
function [x0, x0_og] = koopman_space_conversion(x,y, vx, vy, delta, delta_dot)
    x0 = zeros(13,1);
    x0(1) = x;
    x0(2) = y;
    x0(3) = vx;
    x0(4) = vy;
    x0(5) = delta;
    x0(6) = delta_dot;
    x0(7) = cosd(delta);
    x0(8) = sind(delta);
    x0(9) = x0(3)*x0(7);
    x0(10) = x0(3)*x0(8);
    x0(11) = x0(4)*x0(7);
    x0(12) = x0(4)*x0(8);
    x0(13) = x0(3)*x0(4);
    x0(14) = x0(3)*x0(3);
    x0(15) = x0(4)*x0(4);
    x0_og = zeros(6,1);
    x0_og(1) = x;
    x0_og(2) = y;
    x0_og(3) = vx;
    x0_og(4) = vy;
    x0_og(5) = delta;
    x0_og(6) = delta_dot;
end
