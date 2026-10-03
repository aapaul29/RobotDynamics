function [ C_IC ] = cameraFrameOrientationWithBaseRotation(qW, qX, qY, qZ, q, params)
% Input:
%   qW, qX, qY, qZ: components of the quaternion q_I0 = [qW, qX, qY, qZ],
%                   which describes the orientation from the 0 frame to the
%                   inertial frame
%   q: a 3x1 vector of generalized coordinates
%   params: a struct of parameters
% Output:
%   C_IC: rotation matrix describing the camera frame orientation
%         with respect to the inertial frame when the base orientation is
%         not fixed.


q_I0 = [qW; qX; qY; qZ];
T_0C = T_0C_solution(q, params);
C_0C = T_0C(1:3,1:3);
quat_w = q_I0(1);

% Extract the vector part.
quat_n = q_I0(2:4);

% Map the unit quaternion to a rotation matrix.
C_IO = (2*quat_w^2-1)*eye(3) + 2.0*quat_w*skewMatrix(quat_n) + 2.0*(quat_n*quat_n');


% implement your solution here ...
C_IC = C_IO*C_0C;

end

function A = skewMatrix(q_n)
A = [0, -q_n(3), q_n(2);...
    q_n(3), 0, -q_n(1);...
    -q_n(2), q_n(1), 0];
end