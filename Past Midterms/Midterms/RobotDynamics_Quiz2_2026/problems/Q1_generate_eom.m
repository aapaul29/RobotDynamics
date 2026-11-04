% generate equations of motion
function eom = Q1_generate_eom(gc, kin, params, jac)
% By calling:
%   eom = Q1_generate_eom(gc, kin, params, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion.
%
% Inputs:
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : Struct containing symbolic expressions for the kinematics
%   - params    : Struct with parameters
%   - jac       : Struct containing symbolic expressions for the jacobians
%
% Output:
%   - eom       : Struct with fields {M, b, g}, implementing the system dynamics
%

%% Setup
q = gc.q;                   % Generalized coordinates (4x1 sym)
dq = gc.dq;                 % Generalized velocities (4x1 sym)

R_Ik = cell(4,1);           % Rotation matrices of each body (4x1 cell)->(3x3 sym)
R_Ik{1} = kin.R_Ik{1};      %R_IB
R_Ik{2} = kin.R_Ik{2};      %R_IN
R_Ik{3} = kin.R_Ik{3};      %R_IH
R_Ik{4} = kin.R_Ik{4};      %R_IF

k_I_s = params.k_I_s;       % Inertia tensor of body k in frame k (4x1 cell)->(3x3 double)

m = params.m;               % Mass of body k (4x1 cell)->(1x1 double)

I_g_acc = params.I_g_acc;   % Gravitational acceleration in inertial frame (3x1 double)

I_J_p = jac.I_Jp;            % CoM Positional Jacobian in frame I (4x1 cell)->(3x4 sym)
I_J_r = jac.I_Jr;            % CoM Rotational Jacobian in frame I (4x1 cell)->(3x4 sym)

num_bodies = length(q);     % Number of bodies in the robot, every generalized coordinate moves one body

eom.M = sym(zeros(4,4));
eom.g = sym(zeros(4,1));
eom.b = sym(zeros(4,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = sym(zeros(4,4));  % this accounts for the full robot

for i = 1:num_bodies
    % TODO: implement here
    I_I_s = R_Ik{i} * k_I_s{i} * R_Ik{i}';
    M = M + I_J_p{i}'*m{i}*I_J_p{i} +  I_J_r{i}'*I_I_s*I_J_r{i};
end
M = simplify(M);
fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
g = sym(zeros(4,1));

for j = 1:num_bodies
    % TODO: implement here
    g = g - m{j} * I_J_p{j}' * I_g_acc;
end
g = simplify(g); % Allow more time for more simplified solution
fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(4,1));
for k = 1:num_bodies
    % TODO: implement here
    I_I_s = R_Ik{k} * k_I_s{k} * R_Ik{k}';
    Omega = I_J_r{k} * dq;
    b = b + I_J_p{k}'*m{k}*dAdt(I_J_p{k},q,dq)*dq + I_J_r{k}'*(I_I_s*dAdt(I_J_r{k},q,dq)*dq + cross(Omega,I_I_s*Omega));
end

b = simplify(b);
fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end
