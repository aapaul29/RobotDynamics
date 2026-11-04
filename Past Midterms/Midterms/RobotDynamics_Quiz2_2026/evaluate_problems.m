% Checks that your functions run and that their outputs have the right size.
% This script does NOT test if your solutions are correct.

% Initialize the workspace.
cd(fileparts(mfilename('fullpath')));
run(fullfile(pwd, 'init_workspace.m'));

%% Setup
% generalized coordinates
gc = generate_gc();
q_eval = [0.3, 0.5, -0.4, 0.35]';
dq_eval = [0.1, 0.1, 0.1, 0.1]';

% Initialize the parameters for the mid-term exam.
params = init_params();

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);


%% Q1.
disp('==============');
disp('Running Q1 ...');

try
    eom = Q1_generate_eom(gc, kin, params, jac);
    disp('eom generated.');

    disp('Running eom.M ...');
    M = double(subs(eom.M, gc.q, q_eval));
    disp('Running eom.g ...');
    g = double(subs(eom.g, gc.q, q_eval));
    disp('Running eom.b ...');
    b = double(subs(eom.b, [gc.q, gc.dq], [q_eval, dq_eval]));

    disp('Done.');

    if ~any(M(:)) || ~any(b(:)) || ~any(g(:))
        warning('Some of the returned matrices are empty')
    end
    if any(size(M) ~= [4, 4]) || any(size(g) ~= [4, 1]) || any(size(b) ~= [4, 1])
        warning('M should be 4x4, g and b should be 4x1')
    end

catch ME
    warning('Please check your implementation!');
    disp('The following errors occured:');
    disp(ME.message);
    disp(ME.stack(1));
end

%% Q2.
disp('==============');
disp('Running Q2 ...');

try
    gc_eval.q = q_eval;
    gc_eval.dq = dq_eval;
    tau_eval = ones(4, 1)*0.0001;
    ddq = Q2_forward_dynamics(gc_eval, tau_eval, params);

    disp('Done.');
    if ~any(ddq(:))
        warning('Returned accelerations are null')
    end
    if any(size(ddq) ~= [4, 1])
        warning('ddq should be 4x1')
    end

catch ME
    warning('Please check your implementation!');
    disp('The following errors occured:');
    disp(ME.message);
    disp(ME.stack(1));
end


%% Q3.
disp('==============');
disp('Running Q3 ...');

try
    gc_eval.q = q_eval;
    gc_eval.dq = dq_eval;
    q_des = q_eval / 2.0;
    dq_des = dq_eval / 2.0;
    user_tau = Q3_gravity_compensation(params, gc_eval, q_des, dq_des);

    disp('Done.');

    if ~any(user_tau(:))
        warning('Returned torques are null')
    end
    if any(size(user_tau) ~= [4, 1])
        warning('tau should be 4x1')
    end

catch ME
    warning('Please check your implementation!');
    disp('The following errors occured:');
    disp(ME.message);
    disp(ME.stack(1));
end


%% Q4.
disp('==============');
disp('Running Q4 ...');

try
    gc_eval.q = q_eval;
    gc_eval.dq = dq_eval;
    T_ID = T_ID_fun(q_eval);
    I_r_IDd = T_ID(1:3, 4) + [0; 0.05; 0.0];
    I_v_Dd = [0; 0.2; 0];
    I_a_Dd = [0; 1.0; 0];
    C_IDd = [1, 0, 0; 0, cos(-0.1), -sin(-0.1); 0, sin(-0.1), cos(-0.1)];
    user_tau = Q4_inverse_dynamics_control(params, gc_eval, I_r_IDd, I_v_Dd, I_a_Dd, C_IDd);

    disp('Done.');

    if ~any(user_tau(:))
        warning('Returned torques are null')
    end
    if any(size(user_tau) ~= [4, 1])
        warning('tau should be 4x1')
    end

catch ME
    warning('Please check your implementation!');
    disp('The following errors occured:');
    disp(ME.message);
    disp(ME.stack(1));
end
