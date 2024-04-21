%% 
close all;clear;clc;
%% initialization
sample_number = 150000;
dt = 0.1;
Vx_range = [15;25];                     % range of Vx
Vy_range = [-5;5];                      % range of Vy
delta_range = [-15;15];                 % range of delta
delta_dot_range = [-50;50];             % range of deltadot

f_range = [0.1;5];                      % range of f
theta_range = [-10; 10];                % range of theta
% array
Vx_arr = zeros(sample_number, 1);
Vy_arr = zeros(sample_number, 1);
delta_arr = zeros(sample_number, 1);
delta_dot_arr = zeros(sample_number, 1);
f_arr = zeros(sample_number, 1);
theta_arr = zeros(sample_number, 1);
% next states
state_next = zeros(sample_number, 4);
%%  random states and input
tic;
parfor i = 1:sample_number
    % uniform random
    Vx = unifrnd(Vx_range(1), Vx_range(2));
    Vy = unifrnd(Vy_range(1), Vy_range(2));
    delta = unifrnd(delta_range(1), delta_range(2));
    delta_dot = unifrnd(delta_dot_range(1), delta_dot_range(2));
    f = unifrnd(f_range(1), f_range(2));
    theta = unifrnd(theta_range(1), theta_range(2));
    
    Vx_arr(i) = Vx;
    Vy_arr(i) = Vy;
    delta_arr(i) = delta;
    delta_dot_arr(i) = delta_dot;
    f_arr(i) = f;
    theta_arr(i) = theta;
    
    states = [Vx;Vy;delta;delta_dot];
    inputs = [f; theta];
    % simulate with original continuous-time function to get next states
    [~,x_temp] = ode45(@(t,x) original_state_fcn(x,inputs) , [0 dt], states);
    state_next(i, :) = x_temp(end,:)';
i
end
toc;
%% save to file
save("sindy_data.mat");