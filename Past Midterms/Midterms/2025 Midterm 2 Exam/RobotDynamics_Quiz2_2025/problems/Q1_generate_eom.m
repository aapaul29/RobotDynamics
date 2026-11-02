% generate equations of motion
function eom = Q1_generate_eom(gc, kin, params, jac)
% By calling:
%   eom = generate_eom(gc, kin, params, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion.
%
% Inputs:
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : Struct containing symbolic expresses for the kinematics
%   - params    : Struct with parameters
%   - jac       : Struct containing symbolic expresses for the jacobians
%
% Output:
%   - eom       : Struct with fields {M, b, g}, implementing the system dynamics
%

%% Setup
q = gc.q;                   % Generalized coordinates (8x1 sym)
dq = gc.dq;                 % Generalized velocities (8x1 sym)

R_Ik = cell(7,1);           % Rotation matrices of each body (7x1 cell)->(3x3 sym)
R_Ik{1} = kin.R_Ik{1};      %R_IB
R_Ik{2} = kin.R_Ik{2};      %R_I2
R_Ik{3} = kin.R_Ik{3};      %R_I4
R_Ik{4} = kin.R_Ik{4};      %R_I6
R_Ik{5} = kin.R_Ik{7};      %R_I3
R_Ik{6} = kin.R_Ik{8};      %R_I5
R_Ik{7} = kin.R_Ik{9};      %R_I7

k_I_s = params.k_I_s;       % Inertia tensor of body k in frame k (7x1 cell)->(3x3 double)

m = params.m;               % Mass of body k (7x1 cell)->(1x1 double)

I_g_acc = params.I_g_acc;   % Gravitational acceleration in inertial frame (3x1 double)

I_Jp = jac.I_Jp;            % CoM Positional Jacobian in frame I (7x1 cell)->(3x8 sym)
I_Jr = jac.I_Jr;            % CoM Rotational Jacobian in frame I (7x1 cell)->(3x8 sym)

num_bodies = length(q) - 1;  % Number of bodies in the robot, the first body has two generalized coordinates

eom.M = sym(zeros(8,8));
eom.g = sym(zeros(8,1));
eom.b = sym(zeros(8,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = sym(zeros(8,8));  % this accounts for the full body

for k = 1:num_bodies
    I_I_s = R_Ik{k} * k_I_s{k} * R_Ik{k}';
    M = M + I_Jp{k}'*m{k}*I_Jp{k} + I_Jr{k}'*I_I_s*I_Jr{k};
end
M = simplify(M);
fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
g = sym(zeros(8,1));

for k = 1:num_bodies
    g = g - m{k} * I_Jp{k}' * I_g_acc;
end
g = simplify(g); % Allow more time for more simplified solution
fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(8,1));
for k = 1:num_bodies
    I_I_s = R_Ik{k} * k_I_s{k} * R_Ik{k}';
    Omega = I_Jr{k} * dq;
    b = b + I_Jp{k}'*m{k}*dAdt(I_Jp{k},q,dq)*dq + I_Jr{k}' * (I_I_s*dAdt(I_Jr{k},q,dq)*dq + cross(Omega, I_I_s*Omega));
end

b = simplify(b);
fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end