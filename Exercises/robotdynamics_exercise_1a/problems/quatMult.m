function q_AC = quatMult(q_AB,q_BC)
% Input: two quaternions to be multiplied
% Output: output of the multiplication

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
q_AB_matrix = [q_AB(1), -q_AB(2), -q_AB(3), -q_AB(4);
    q_AB(2), q_AB(1), -q_AB(4), q_AB(3);
    q_AB(3), q_AB(4), q_AB(1), -q_AB(2);
    q_AB(4), -q_AB(3), q_AB(2), q_AB(1)];

q_BC_vector = [q_BC(1);
    q_BC(2);
    q_BC(3);
    q_BC(4)];

q_AC = q_AB_matrix * q_BC_vector;
end

