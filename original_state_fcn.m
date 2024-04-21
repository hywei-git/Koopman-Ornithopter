function state_derivatives = original_state_fcn(states, inputs) % Vx, Vy, delta % f, theta
S = 0.419;                                        % wing surface
b = 0.75;                                         % wing span
c = 0.317;                                        % wing mean chord
rho = 1.01;                                       % air density
beta_max = 30;                                    % maximum angle of flapping wing
l = 0.3;                                          % distance between center of mass of ornithopter to center of tail
St = 0.091;                                       % tail surface
bt = 0.46;                                        % tail wingspan
ARt = bt^2/St;                                    % tail aspect ratio
g = 9.8;                                          % gravity constant
m = 3.415;                                        % total mass
I = (4*l^2+0.01)*m/12;                            % Inertia

U = sqrt(states(1)^2+states(2)^2);                % relative velocity
gamma = atand(states(2)/states(1));               % fly path angle
phi = states(3)-gamma;                            % angle of attack
f = inputs(1);                                    % wing flapping frequency
theta = inputs(2);                                % tail wing angle

Re = U*c/1.5111/10^-5;                            % Reynolds number
% integral of vertical forces from flapping wings over wing length and a flapping cycle (body frame)
Vertical_wing = integral2(@(r, t) vertical(r, t, f, b, c, U, phi, beta_max, rho, Re), 0, b, 0, 1/f);
% integral of horizontal forces from flapping wings over wing length and a flapping cycle (body frame)
Horizontal_wing = integral2(@(r, t) horizontal(r, t, f, b, c, U, phi, beta_max, rho, Re), 0, b, 0, 1/f);
Vertical_wing = Vertical_wing*2*f; % average over a cycle
Horizontal_wing = Horizontal_wing*2*f; % average over a cycle
F_wing = [Vertical_wing Horizontal_wing];

% vertical and horizontal forces from tail (body frame)
F_tail = Forces_tail(U, gamma, states(3), theta, rho, St, ARt, bt);
delta_dot = states(4);% pitch angular velocity
delta_dotdot = -F_tail(1, 1)*l/I*180/pi; % pitch angular acceleration

F_total = F_wing + F_tail; % summing the forces from flapping wings and tail

% turning forces into inertial frame
F_horizontal_inertia = F_total(1, 2)*cosd(states(3)) - F_total(1, 1)*sind(states(3));
F_vertical_inertia = F_total(1, 2)*sind(states(3)) + F_total(1, 1)*cosd(states(3));

% Newton's Law
Vx_dot = F_horizontal_inertia/m;
Vy_dot = F_vertical_inertia/m - g;
% return state derivatives
state_derivatives = [Vx_dot;Vy_dot;delta_dot;delta_dotdot];

end

