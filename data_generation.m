%% 
close all;clear;clc;
%% load file
load("sindy_data.mat");
%% next states
Vx_next_arr = state_next(:, 1);
Vy_next_arr = state_next(:, 2);
delta_next_arr = state_next(:, 3);
delta_dot_next_arr = state_next(:, 4);
%% Koopman states
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
state_matrix(:, 12) = Vx_arr.*Vx_arr;
state_matrix(:, 13) = Vy_arr.*Vy_arr;
%% next Koopman states
state_next_matrix = zeros(sample_number, 11);
state_next_matrix(:, 1:4) = state_next;
state_next_matrix(:, 5) = cosd(state_next(:, 3));
state_next_matrix(:, 6) = sind(state_next(:, 3));
state_next_matrix(:, 7) = state_next(:, 1).*cosd(state_next(:, 3));
state_next_matrix(:, 8) = state_next(:, 1).*sind(state_next(:, 3));
state_next_matrix(:, 9) = state_next(:, 2).*cosd(state_next(:, 3));
state_next_matrix(:, 10) = state_next(:, 2).*sind(state_next(:, 3));
state_next_matrix(:, 11) = state_next(:, 1).*state_next(:, 2);
state_next_matrix(:, 12) = state_next(:, 1).*state_next(:, 1);
state_next_matrix(:, 13) = state_next(:, 2).*state_next(:, 2);
%% regression to fit next Koopman states with Koopman states
A = zeros(13, 13);
B = zeros(13, 2);
for i = 1:13
    state_input_matrix = [state_matrix f_arr theta_arr];
    theta = pinv(state_input_matrix'*state_input_matrix)*state_input_matrix'*state_next_matrix(:, i);
    theta = theta';
    A(i, :) = theta(1, 1:13);
    B(i, :) = theta(1, 14:15);
end
%% add two states x, y
A = [[1 0;0 1] [dt 0;0 dt] zeros(2, 11);zeros(13, 2) A];
B = [zeros(2, 2);B];
%% save linear system matrix A, B
save("linear_model.mat", "A", "B");