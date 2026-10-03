function B_r = rotVecWithQuat(q_BA,A_r)
% Input: the orientation quaternion and the coordinate of the vector to be mapped
% Output: the coordinates of the vector in the target frame

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
rotation_matrix = quatToRotMat(q_BA);
B_r = rotation_matrix * A_r;
end
