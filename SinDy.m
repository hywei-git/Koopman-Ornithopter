%% 
close all;clear;clc;
%% initialization
sample_number = 150000;
dt = 0.1;
% % range
% Vx_range = [0.1;5];
% Vy_range = [-2;2];
% delta_range = [-30;30];
% delta_dot_range = [-100;100];
% 
% f_range = [0.1;5];
% theta_range = [-5; 5];
% range
Vx_range = [15;25];
Vy_range = [-5;5];
delta_range = [-15;15];
delta_dot_range = [-50;50];

f_range = [0.1;5];
theta_range = [-10; 10];
% array
Vx_arr = zeros(sample_number, 1);
Vy_arr = zeros(sample_number, 1);
delta_arr = zeros(sample_number, 1);
delta_dot_arr = zeros(sample_number, 1);
f_arr = zeros(sample_number, 1);
theta_arr = zeros(sample_number, 1);

state_next = zeros(sample_number, 4);
%%  random states and input
tic;
parfor i = 1:sample_number
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
    
    [~,x_temp] = ode45(@(t,x) original_state_fcn(x,inputs) , [0 dt], states);
    state_next(i, :) = x_temp(end,:)';
i
end
toc;
%%
Vx_next_arr = state_next(:, 1);
Vy_next_arr = state_next(:, 2);
delta_next_arr = state_next(:, 3);
delta_dot_next_arr = state_next(:, 4);

%% state extension
state_matrix = zeros(sample_number, 11);
state_matrix(:, 1) = Vx_arr;
state_matrix(:, 2) = Vy_arr;
%state_matrix(:, 3) = delta_arr;
%state_matrix(:, 4) = delta_dot_arr;
%state_matrix(:, 5) = cosd(delta_arr);
%state_matrix(:, 6) = sind(delta_arr);
state_matrix(:, 3) = Vx_arr.*cosd(delta_arr);
state_matrix(:, 4) = Vx_arr.*sind(delta_arr);
state_matrix(:, 5) = Vy_arr.*cosd(delta_arr);
state_matrix(:, 6) = Vy_arr.*sind(delta_arr);
state_matrix(:, 7) = Vx_arr.*Vy_arr;
state_matrix(:, 8) = Vx_arr.^2;
state_matrix(:, 9) = Vy_arr.^2;
%state_matrix(:, 14) = cosd(theta_arr);
%state_matrix(:, 15) = sind(theta_arr);
state_matrix(:, 10) = Vx_arr.*cosd(theta_arr);
state_matrix(:, 11) = Vx_arr.*sind(theta_arr);
state_matrix(:, 12) = Vy_arr.*cosd(theta_arr);
state_matrix(:, 13) = Vy_arr.*sind(theta_arr);
state_matrix(:, 14) = f_arr;
state_matrix(:, 15) = f_arr.*cosd(delta_arr);
state_matrix(:, 16) = f_arr.*sind(delta_arr);
state_matrix(:, 17) = ones(sample_number, 1);
%% scaling
state_matrix(:, 3) = state_matrix(:, 3)./1;
state_matrix(:, 4) = state_matrix(:, 4)./1;
%% state derivative extension
state_next_matrix = zeros(sample_number, 4);
state_next_matrix(:, 1:4) = state_next;
%% scaling
state_next_matrix(:, 3) = state_next_matrix(:, 3)./1;
state_next_matrix(:, 4) = state_next_matrix(:, 4)./1;
%% regression
A = zeros(4, 17);
for i = 1:3
    theta = pinv(state_matrix'*state_matrix)*state_matrix'*state_next_matrix(:, i);
    theta = theta';
    A(i, :) = theta;
end

state_matrix4 = [state_matrix(:, 1:6),state_matrix(:, 10:13), state_matrix(:, 17)];
theta4 = (pinv(state_matrix4'*state_matrix4)*state_matrix4'*state_next_matrix(:, 4))';
A(4, 1:6) = theta4(1:6);
A(4, 10:13) = theta4(7:10);
A(4, 17) = theta4(11);

%% add two states x, y
A = [[dt 0;0 dt] zeros(2, 15);A];
%% save linear system matrix A, B
save("nonlinear_simplified_model.mat", "A");
%% clear function persistent
clear nonlinear_simplified_state_fcn;