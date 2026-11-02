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
q = gc.q;      % Generalized coordinates (6x1 sym)
dq = gc.dq;    % Generalized velocities (6x1 sym)

T_Ik = kin.T_Ik(1:4);        % Homogeneous transforms (4x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik(1:4);        % Rotation matrices (4x1 cell)->(3x3 sym)

k_I_s = params.k_I_s;      % Inertia tensor of body k in frame k (4x1 cell)->(3x3 double)
m = params.m;              % Mass of body k (4x1 cell)->(1x1 double)
I_g_acc = params.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = params.k_r_ks;    % CoM location of body k in frame k (4x1 cell)->(3x1 double)

I_Jp = jac.I_Jp;    % CoM Positional Jacobian in frame I (4x1 cell)->(3x6 sym)
I_Jr = jac.I_Jr;    % CoM Rotational Jacobian in frame I (4x1 cell)->(3x6 sym)

num_bodies = length(q) - 2;  % Number of bodies in the robot

eom.M = sym(zeros(6,6));
eom.g = sym(zeros(6,1));
eom.b = sym(zeros(6,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = sym(zeros(6,6));  % this accounts for the full body

for k = 1:num_bodies
    I_I_s = R_Ik{k} * k_I_s{k} * R_Ik{k}';
    M = M + I_Jp{k}'*m{k}*I_Jp{k} +  I_Jr{k}'*I_I_s*I_Jr{k};
end
% TODO: implement here

M = simplify(M);
fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
g = sym(zeros(6,1));

for k = 1:num_bodies
    g = g - m{k} * I_Jp{k}' * I_g_acc;
    
    
end
g = simplify(g); % Allow more time for more simplified solution
fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(6,1));
for k = 1:num_bodies
    I_I_s = R_Ik{k} * k_I_s{k} * R_Ik{k}';
    Omega = I_Jr{k} * dq;
    b = b + I_Jp{k}'*m{k}*dAdt(I_Jp{k},q,dq)*dq + I_Jr{k}'*(I_I_s*dAdt(I_Jr{k},q,dq)*dq + cross(Omega,I_I_s*Omega));
    % TODO: implement here
end
b = simplify(b);



fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end