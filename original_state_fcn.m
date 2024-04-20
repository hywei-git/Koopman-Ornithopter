function state_derivatives = original_state_fcn(states, inputs) % Vx, Vy, delta % f, theta
S = 0.419; % wing surface
b = 0.75; % wing span
c = 0.317; % wing mean chord
rho = 1.01;
beta_max = 30;
l = 0.3;
St = 0.091; % tail surface
bt = 0.46; % tail wingspan
ARt = bt^2/St; % tail aspect ratio
g = 9.8;
m = 3.415;
% I = 0.009;
I = (4*l^2+0.01)*m/12;

% b = 0.35;                                   % half of wing span (單翅長)
% c = 0.2;                                    % wing chord
% beta_max = 45;                              % maximum flapping angle
% rho = 1.225;                                % air density
% m = 1.7;                                    % Mass
% states(3) = 0;

U = sqrt(states(1)^2+states(2)^2);
gamma = atand(states(2)/states(1));
phi = states(3)-gamma;
f = inputs(1);
theta = inputs(2);

Re = U*c/1.5111/10^-5;
Vertical_wing = integral2(@(r, t) vertical(r, t, f, b, c, U, phi, beta_max, rho, Re), 0, b, 0, 1/f);
Horizontal_wing = integral2(@(r, t) horizontal(r, t, f, b, c, U, phi, beta_max, rho, Re), 0, b, 0, 1/f);
Vertical_wing = Vertical_wing*2*f;
Horizontal_wing = Horizontal_wing*2*f;
F_wing = [Vertical_wing Horizontal_wing];

F_tail = Forces_tail(U, gamma, states(3), theta, rho, St, ARt, bt);
% F_tail = [0 0];
delta_dot = states(4);
delta_dotdot = -F_tail(1, 1)*l/I*180/pi;

F_total = F_wing + F_tail;

F_horizontal_inertia = F_total(1, 2)*cosd(states(3)) - F_total(1, 1)*sind(states(3));
F_vertical_inertia = F_total(1, 2)*sind(states(3)) + F_total(1, 1)*cosd(states(3));

Vx_dot = F_horizontal_inertia/m;
Vy_dot = F_vertical_inertia/m - g;

state_derivatives = [Vx_dot;Vy_dot;delta_dot;delta_dotdot];

end

