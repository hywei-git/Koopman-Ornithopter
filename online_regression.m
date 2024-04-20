function [updated_A, updated_B] = online_regression(Ad, Bd, states, inputs)
    updated_A = zeros(15, 15);
    updated_B = zeros(15, 2);
    state_input = [states;inputs];
    for i = 1:15
        theta = pinv(state_input(:, 1:end-1)*state_input(:, 1:end-1)')*state_input(:, 1:end-1)*states(i, 2:end)';
        updated_A(i, :) = theta(1:15, 1)';
        updated_B(i, :) = theta(16:17, 1)';
    end 
% updated_B = Bd;
%     for i = 1:13
%         theta = pinv(inputs(:, 1:end-1)*inputs(:, 1:end-1)')*inputs(:, 1:end-1)*...
%             (states(i, 2:end)' - (Ad(i, :)*states(:, 1:end-1))');
%         updated_B(i, :) = theta';
%     end
        
end

