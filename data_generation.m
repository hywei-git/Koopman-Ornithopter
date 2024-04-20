%% 
close all;clear;clc;
% %% initialization
% sample_number = 150000;
% dt = 0.1;
% % % range
% % Vx_range = [0.1;5];
% % Vy_range = [-2;2];
% % delta_range = [-30;30];
% % delta_dot_range = [-100;100];
% % 
% % f_range = [0.1;5];
% % theta_range = [-5; 5];
% % range
% Vx_range = [15;25];
% Vy_range = [-5;5];
% delta_range = [-15;15];
% delta_dot_range = [-50;50];
% 
% f_range = [0.1;5];
% theta_range = [-10; 10];
% % array
% Vx_arr = zeros(sample_number, 1);
% Vy_arr = zeros(sample_number, 1);
% delta_arr = zeros(sample_number, 1);
% delta_dot_arr = zeros(sample_number, 1);
% f_arr = zeros(sample_number, 1);
% theta_arr = zeros(sample_number, 1);
% 
% state_next = zeros(sample_number, 4);
% %%  random states and input
% tic;
% parfor i = 1:sample_number
%     Vx = unifrnd(Vx_range(1), Vx_range(2));
%     Vy = unifrnd(Vy_range(1), Vy_range(2));
%     delta = unifrnd(delta_range(1), delta_range(2));
%     delta_dot = unifrnd(delta_dot_range(1), delta_dot_range(2));
%     f = unifrnd(f_range(1), f_range(2));
%     theta = unifrnd(theta_range(1), theta_range(2));
%     
%     Vx_arr(i) = Vx;
%     Vy_arr(i) = Vy;
%     delta_arr(i) = delta;
%     delta_dot_arr(i) = delta_dot;
%     f_arr(i) = f;
%     theta_arr(i) = theta;
%     
%     states = [Vx;Vy;delta;delta_dot];
%     inputs = [f; theta];
%     
%     [~,x_temp] = ode45(@(t,x) original_state_fcn(x,inputs) , [0 dt], states);
%     state_next(i, :) = x_temp(end,:);
% i
% end
% toc;
%%
load("sindy_data.mat");
%%
Vx_next_arr = state_next(:, 1);
Vy_next_arr = state_next(:, 2);
delta_next_arr = state_next(:, 3);
delta_dot_next_arr = state_next(:, 4);
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
state_matrix(:, 12) = Vx_arr.*Vx_arr;
state_matrix(:, 13) = Vy_arr.*Vy_arr;
%% state derivative extension
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
%% scaling
% state_derivative_matrix(:, 3) = state_derivative_matrix(:, 3)./100;
% state_derivative_matrix(:, 4) = state_derivative_matrix(:, 4)./100;
%% regression
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