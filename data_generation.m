%% 
close all;clear;clc;
%% initialization
sample_number = 10000;
% range
Vx_range = [0.1;10];
Vy_range = [-3;3];
delta_range = [-45;45];
delta_dot_range = [-300;300];

f_range = [0;8];
theta_range = [-20;45];
% array
Vx_arr = zeros(sample_number, 1);
Vy_arr = zeros(sample_number, 1);
delta_arr = zeros(sample_number, 1);
delta_dot_arr = zeros(sample_number, 1);
f_arr = zeros(sample_number, 1);
theta_arr = zeros(sample_number, 1);

state_derivatives = zeros(sample_number, 4);
%%  random states and input
for i = 1:sample_number
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
    
    state_derivatives(i, :) = original_state_fcn(states, inputs)';
end
Vx_deri_arr = state_derivatives(:, 1);
Vy_deri_arr = state_derivatives(:, 2);
delta_deri_arr = state_derivatives(:, 3);
delta_dot_deri_arr = state_derivatives(:, 4);

%% state extension
state_matrix = zeros(sample_number, 11);
state_matrix(:, 1) = Vx_arr;
state_matrix(:, 2) = Vy_arr;
state_matrix(:, 3) = delta_arr;
state_matrix(:, 4) = delta_dot_arr;
state_matrix(:, 5) = cosd(delta_arr);
state_matrix(:, 6) = sind(delta_arr);
state_matrix(:, 7) = Vx_arr.*cosd(delta_arr);
state_matrix(:, 8) = Vx_arr.*sind(delta_arr);
state_matrix(:, 9) = Vy_arr.*cosd(delta_arr);
state_matrix(:, 10) = Vy_arr.*sind(delta_arr);
state_matrix(:, 11) = Vx_arr.*Vy_arr;
%% scaling
state_matrix(:, 3) = state_matrix(:, 3);
state_matrix(:, 4) = state_matrix(:, 4);
%% state derivative extension
state_derivative_matrix = zeros(sample_number, 11);
state_derivative_matrix(:, 1:4) = state_derivatives;
state_derivative_matrix(:, 5) = -delta_deri_arr.*sind(delta_arr);
state_derivative_matrix(:, 6) = delta_deri_arr.*cosd(delta_arr);
state_derivative_matrix(:, 7) = state_derivative_matrix(:, 1).*cosd(delta_arr) + Vx_arr.*state_derivative_matrix(:, 5);
state_derivative_matrix(:, 8) = state_derivative_matrix(:, 1).*sind(delta_arr) + Vx_arr.*state_derivative_matrix(:, 6);
state_derivative_matrix(:, 9) = state_derivative_matrix(:, 2).*cosd(delta_arr) + Vy_arr.*state_derivative_matrix(:, 5);
state_derivative_matrix(:, 10) = state_derivative_matrix(:, 2).*sind(delta_arr) + Vy_arr.*state_derivative_matrix(:, 6);
state_derivative_matrix(:, 11) = state_derivative_matrix(:, 1).*Vy_arr + state_derivative_matrix(:, 2).*Vx_arr;

%% scaling
state_derivative_matrix(:, 3) = state_derivative_matrix(:, 3);
state_derivative_matrix(:, 4) = state_derivative_matrix(:, 4);
%% regression
A = zeros(11, 11);
B = zeros(11, 2);
for i = 1:11
    state_input_matrix = [state_matrix f_arr theta_arr];
    theta = pinv(state_input_matrix'*state_input_matrix)*state_input_matrix'*state_derivative_matrix(:, i);
    theta = theta';
    A(i, :) = theta(1, 1:11);
    B(i, :) = theta(1, 12:13);
end
%% add two states x, y
A = [zeros(2, 2) [1 0;0 1] zeros(2, 9);zeros(11, 2) A];
B = [zeros(2, 2);B];
%% save linear system matrix A, B
save("linear_model.mat", "A", "B");
max(max(A))
min(min(A))