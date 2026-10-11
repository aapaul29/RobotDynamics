function [ J_IT_r ] = jointToRotationJacobian(q, params)
% Inputs:
%     q: a 6x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     J_IT_r: rotation Jacobian of tray frame {E} wrt inertia frame {I}
%     expressed in inertial frame {I}

% link lengths (meters)
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;

% Generalized coordinates
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
q4 = q(5);
q5 = q(6);


% Implement your solution here...
J_IT_r = zeros(3,6);

end
