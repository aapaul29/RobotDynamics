function [ J_IE_p ] = jointToPositionJacobian(q, params)
% Inputs:
%     q: a 4x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     J_IE_p: 3x4 position Jacobian of gripper frame {E} wrt inertia frame {I}
%             in inertial frame {I}

% link lengths (meters)
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;
l4 = params.l4;
l5 = params.l5;
l6 = params.l6;
l7 = params.l7;

% Joint positions
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% Implement your solution here...
J_IE_p = zeros(3,4);

end
