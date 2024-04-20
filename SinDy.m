%% 
close all;clear;clc;
%% load file
load("sindy_data.mat");
%% next states
Vx_next_arr = state_next(:, 1);
Vy_next_arr = state_next(:, 2);
delta_next_arr = state_next(:, 3);
delta_dot_next_arr = state_next(:, 4);
state_next_matrix = zeros(sample_number, 4);
state_next_matrix(:, 1:4) = state_next;
%% Koopman states
state_matrix = zeros(sample_number, 11);
state_matrix(:, 1) = Vx_arr;
state_matrix(:, 2) = Vy_arr;
state_matrix(:, 3) = Vx_arr.*cosd(delta_arr);
state_matrix(:, 4) = Vx_arr.*sind(delta_arr);
state_matrix(:, 5) = Vy_arr.*cosd(delta_arr);
state_matrix(:, 6) = Vy_arr.*sind(delta_arr);
state_matrix(:, 7) = Vx_arr.*Vy_arr;
state_matrix(:, 8) = Vx_arr.^2;
state_matrix(:, 9) = Vy_arr.^2;
state_matrix(:, 10) = Vx_arr.*cosd(theta_arr);
state_matrix(:, 11) = Vx_arr.*sind(theta_arr);
state_matrix(:, 12) = Vy_arr.*cosd(theta_arr);
state_matrix(:, 13) = Vy_arr.*sind(theta_arr);
state_matrix(:, 14) = f_arr;
state_matrix(:, 15) = f_arr.*cosd(delta_arr);
state_matrix(:, 16) = f_arr.*sind(delta_arr);
state_matrix(:, 17) = ones(sample_number, 1);
%% regression to fit next states with Koopman states
A = zeros(4, 17);
for i = 1:3
    theta = pinv(state_matrix'*state_matrix)*state_matrix'*state_next_matrix(:, i);
    theta = theta';
    A(i, :) = theta;
end
% select certain extended states to fit deltadotdot
state_matrix4 = [state_matrix(:, 1:6),state_matrix(:, 10:13), state_matrix(:, 17)];
theta4 = (pinv(state_matrix4'*state_matrix4)*state_matrix4'*state_next_matrix(:, 4))';
A(4, 1:6) = theta4(1:6);
A(4, 10:13) = theta4(7:10);
A(4, 17) = theta4(11);
%% add two states x, y
A = [[dt 0;0 dt] zeros(2, 15);A];
%% save parameter matrix A
save("nonlinear_simplified_model.mat", "A");
%% clear function persistent
clear nonlinear_simplified_state_fcn;