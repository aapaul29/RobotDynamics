function R = quatToRotMat(q)
% Input: quaternion [w x y z]
% Output: corresponding rotation matrix

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
% Normalize Quaternion
q = q / norm(q);
% Cross Product Quaternion Matrix
q_cross = [0, -q(4), q(3);
    q(4), 0, -q(2);
    -q(3), q(2), 0];
% Compute Rotation Matrix
R = eye(3, 3) + 2 * q(1) * q_cross + 2 * q_cross * q_cross;
end
