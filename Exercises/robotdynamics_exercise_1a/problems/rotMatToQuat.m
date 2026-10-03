function q = rotMatToQuat(R)
% Input: rotation matrix
% Output: corresponding quaternion [w x y z]

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
angle = acos((R(1,1) + R(2,2) + R(3,3) - 1) / 2);
% Normal Vector
n = 1 / (2 * sin(angle)) * [R(3,2) - R(2,3);
    R(1,3) - R(3,1);
    R(2,1) - R(1,2)];
  % Real Part
  xsi_0 = cos(angle / 2);
  % Imaginary Part
  xsi_hat = sin(angle / 2) * n;
  
  q = [xsi_0;
    xsi_hat];
  
end