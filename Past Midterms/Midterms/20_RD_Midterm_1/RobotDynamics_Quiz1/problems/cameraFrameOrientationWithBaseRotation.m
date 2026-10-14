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



% implement your solution here ...
% Find C_I0
C_I0 = [(qW^2 + qX^2 - qY^2 - qZ^2), (2* qX *qY - 2 * qW * qZ), (2 * qW * qY + 2 * qX * qZ);
    (2 * qW * qZ + 2 * qX * qY), (qW^2 - qX^2 + qY^2 - qZ^2), (2 * qY * qZ - 2 * qW * qX);
    (2 * qX * qZ - 2 * qW * qY), (2 * qW * qX + 2 * qY * qZ), (qW^2 - qX^2 - qY^2 + qZ^2)];



% Extract C_0C
T_0C = T_0C_solution(q, params);
C_0C = T_0C(1:3, 1:3);

% Compute C_IC
C_IC = C_I0 * C_0C;

end
