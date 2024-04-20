clear all; close all; clc;
%%
computation_time = 0;
N = 5;                 % Horizon length
T = 0.1;                % MPC step size
t_end = 30;             % Simulation length
t = (0:T:t_end)';
dim_state = 15;         % Dimension of state
dim_input = 2;          % Dimension of input
x_length = dim_state*N; % Length of stacked state vector
u_length = dim_input*N; % Length decision variable vector

load("linear_model.mat");

Ad = A;                                     % Discretized A
Bd = B;                                     % Discretized B
Vx_ref = 25;                                % Reference Vx
Vy_ref = 0;                                 % Reference Vy
y_ref = [20*ones(1,100) 25*ones(1,100) 20*ones(1,150)];% 25*ones(1,100) 20*ones(1,100) 25*ones(1,150)];
%y_ref = [20*ones(1,length(t)+100)];
%y_ref = 15 - 5*cosd(2*(0:650));
w_vx = 1;
w_vy = 10;
w_y = 100;
%% Setting up weight matrices
Q = zeros(dim_state);                                          % Original weight on x^2
Q(3,3) = w_vx;                                                 % Penalizes Vx
Q(2,2) = w_y;
%Q(4,4) = w_vy;                                                 % Penalizes Vy
R = zeros(dim_input);                                           % Original weight on u^2
R(2, 2) = 1;
P = zeros(1,dim_state);
P(3) = -2*w_vx*Vx_ref;
P(2) = -2*w_y*y_ref(1);
[x0, x0_og] = get_initial_states(0,15,15,0,10,0);               % Initial state x0 = get_initial_states(x,y, vx, vy, delta, delta_dot)


%%%%%%%%%%%% Change this to equilibrium u for better convergence %%%%%%%%%%%%
u0 = ones(u_length,1);                      % Initial guess for input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
for i = 0:N-1
    u_min_vec(i*dim_input+1:i*dim_input+dim_input) = u_min;
    u_max_vec(i*dim_input+1:i*dim_input+dim_input) = u_max;
end
Aineq = [-eye(u_length); eye(u_length)];
bineq = [-u_min_vec; u_max_vec];

%% Online regression
online_regression_on = true;
integral_feedback_on = false;
%% Start Simulation
error = 0;
u_next = [0.1; 0];
x = zeros(dim_state, t_end/T + 1);
x_og = zeros(6, t_end/T + 1);
x(:,1) = x0;
x_og(:,1) = x0_og;
u = u_next;
tic;
for iter = 1:(t_end/T)
    tStart = tic;
    % Updating r_vec and constraints before next step
    r_vec = zeros(1, u_length);
    for i = 0:N-1
        r_vec = r_vec + 2*(x(:,iter)'*(Ad^(i+1))'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:)) + P*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
    end
    u_next = NextInputLMPC(Q_bar, r_vec, Aineq, bineq, u0, dim_input);
    u_next = u_next+integral_feedback_on*0.4*[error;0]; % 0.4
    u_next = min([5;10], max([0.1;-10], u_next));
    tEnd = toc(tStart);
    % update computation time
    computation_time = computation_time + tEnd;
    [t_temp,x_og_temp] = ode45(@(t,x) original_state_fcn(x,u_next) , [0 T], x_og(3:end,iter));
    error = error + Vx_ref - x_og_temp(end,1);
    error = min(3, max(-6, error));
    x_og(1:2,iter+1) = x_og(1:2,iter) + T*x_og(3:4,iter);
    x_og(3:end,iter+1) = x_og_temp(end,:)';

    [x(:,iter+1), ~] = get_initial_states(x_og(1,iter+1),x_og(2,iter+1),x_og(3,iter+1),...
                                          x_og(4,iter+1),x_og(5,iter+1),x_og(6,iter+1));
    u = [u u_next];
    
    
    P = zeros(1,dim_state);
    P(3) = -2*w_vx*Vx_ref;
    Q_bar = zeros(u_length);
    r_vec = zeros(1, u_length);
    for i = 0:N-1
        P(2) = -2*w_y*y_ref(iter+i);
        Q_bar = Q_bar + H_AB(i*dim_state+1:i*dim_state+dim_state,:)'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
        r_vec = r_vec + 2*(x0'*(Ad^(i+1))'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:)) + P*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
        Q_bar(i*dim_input+1:i*dim_input+dim_input,i*dim_input+1:i*dim_input+dim_input) = Q_bar...
            (i*dim_input+1:i*dim_input+dim_input,i*dim_input+1:i*dim_input+dim_input) + R;
    end
    
    
    
    % update linear model
    if rem(iter, 75) == 0 && online_regression_on
        [updated_A, updated_B] = online_regression(Ad, Bd, x(:, 1:iter-1), u(:, 1:iter-1));
        Ad = updated_A*0.1 + Ad*0.9;
        Bd = updated_B*0.1 + Bd*0.9;
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
            r_vec = r_vec + 2*(x(:,iter)'*(Ad^(i+1))'*Q*H_AB(i*dim_state+1:i*dim_state+dim_state,:)) + P*H_AB(i*dim_state+1:i*dim_state+dim_state,:);
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

%% save to file
if online_regression_on && integral_feedback_on
    save("Linear_MPC_online_regression_integral_feedback_data.mat", 'x', 'u', 'computation_time');
elseif online_regression_on && ~integral_feedback_on
    save("Linear_MPC_online_regression_data.mat", 'x', 'u', 'computation_time');
else
    save("Linear_MPC_data.mat", 'x', 'u', 'computation_time');
end
%% Compute quadprog
function u_next = NextInputLMPC(Q_bar, r_vec, Aineq, bineq, u0, dim_input)
options = optimoptions('quadprog', 'Algorithm', 'active-set','ObjectiveLimit',-1e20);
[u_con, J_con] = quadprog(2*Q_bar, r_vec, Aineq, bineq, [], [], [], [], u0, options);
J_con;
% [u_con, J_con] = quadprog(2*Q_bar, r_vec, [], [], [], [], -bineq(1:20), bineq(21:end), u0, options);
u_next = u_con(1:dim_input);
end

function [x0, x0_og] = get_initial_states(x,y, vx, vy, delta, delta_dot)
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
