function [ q_iter ] = inverseKinematics( p_des, params )  
    % Inputs:
    %  p_des         : desired gripper pose (3x1)
    %  params        : a struct of parameters

    % Output:
    %  q_iter        : joint position command (3x1)
    
    % Choose a pseudo_inverse damping coefficient
    lambda = 1e-2;
    % Choose a convergence threshold
    epsilon = 1e-3;
    % Maximum number of iterations
    N_max = 100;
    % initialize the IK
    q_iter = [1; 1; 1];
    it = 0;
    dxe = Inf;
    % 2. Iterate until terminating condition.
    while (it==0 || (norm(dxe)>epsilon && it < N_max))
        % 3. evaluate Jacobian for current q
        I_J = jointToGripperAnalyticalJacobian_solution(q_iter,params);
        
        % 4. Update the psuedo inverse
        I_J_pinv = pseudoInverseMat_solution(I_J, lambda);
        
        % 5. Find the end-effector configuration error vector
        % position error
        I_r_IE = jointTo2DGripperPosition_solution(q_iter);
        dr = p_des - I_r_IE; 
        % rotation error
        %C_IE = jointToRotMat_solution(q);
        %C_err = C_IE_des*C_IE';
        %dph = rotMatToRotVec_solution(C_err); 
        % 6D error

        dxe = dr;
        
        % 6. Update the generalized coordinates
        q_iter = q_iter + I_J_pinv*dxe;
         
        
        it = it+1;
    end


end