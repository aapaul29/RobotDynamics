function J_R = jointToRotJac(q)
% Input: vector of generalized coordinates (joint angles)
% Output: Jacobian of the end-effector orientation which maps joint
% velocities to end-effector angular velocities in I frame.

% Access all Transformation Matrices
T_I0 = getTransformI0_solution();
T_01 = jointToTransform01_solution(q(1));
T_12 = jointToTransform12_solution(q(2));
T_23 = jointToTransform23_solution(q(3));
T_34 = jointToTransform34_solution(q(4));
T_45 = jointToTransform45_solution(q(5));
T_56 = jointToTransform56_solution(q(6));
T_6E = getTransform6E_solution();

% Transformation Matrices w.r.t. I
T_I1 = T_I0 * T_01;
T_I2 = T_I1 * T_12;
T_I3 = T_I2 * T_23;
T_I4 = T_I3 * T_34;
T_I5 = T_I4 * T_45;
T_I6 = T_I5 * T_56;
T_IE = T_I6 * T_6E;

% Extract Rotation Matrices
C_I0 = T_I0(1:3, 1:3);
C_I1 = T_I1(1:3, 1:3);
C_I2 = T_I2(1:3, 1:3);
C_I3 = T_I3(1:3, 1:3);
C_I4 = T_I4(1:3, 1:3);
C_I5 = T_I5(1:3, 1:3);
C_I6 = T_I6(1:3, 1:3);
C_IE = T_IE(1:3, 1:3);


% Normal Unit Vectors
n_1 = [0, 0, 1]';
n_2 = [0, 1, 0]';
n_3 = [0, 1, 0]';
n_4 = [1, 0, 0]';
n_5 = [0, 1, 0]';
n_6 = [1, 0, 0]';

% Normal vectors in I frame
I_n_1 = C_I1 * n_1;
I_n_2 = C_I2 * n_2;
I_n_3 = C_I3 * n_3;
I_n_4 = C_I4 * n_4;
I_n_5 = C_I5 * n_5;
I_n_6 = C_I6 * n_6;

% Compute the rotational jacobian.
J_R = [I_n_1, I_n_2, I_n_3, I_n_4, I_n_5, I_n_6];

end
