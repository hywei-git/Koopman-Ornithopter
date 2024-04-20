function state_next = nonlinear_simplified_state_fcn(states, inputs)
    persistent nonlinear_A;
    if isempty(nonlinear_A)
        parameters = load('nonlinear_simplified_model.mat');
        nonlinear_A = parameters.A;
    end
    Vx = states(3);
    Vy = states(4);
    delta = states(5);
    delta_dot = states(6);
    f = inputs(1);
    theta = inputs(2);
    
    state_next = nonlinear_A*[Vx, Vy, Vx*cosd(delta), Vx*sind(delta),...
        Vy*cosd(delta), Vy*sind(delta), Vx*Vy, Vx^2, Vy^2,...
        Vx*cosd(theta), Vx*sind(theta), Vy*cosd(theta), Vy*sind(theta), f, f*cosd(theta), f*sind(theta), 1]';
    state_next(1:2) = state_next(1:2) + [states(1); states(2)];
%     state_derivatives(5) = delta_dot;
end

