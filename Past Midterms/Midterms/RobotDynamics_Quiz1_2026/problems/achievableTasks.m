function [ achievable ] = achievableTasks( stuck_idx )
  % Inputs:
  %  stuck_idx     : index of the stuck joint in q
  %                  (1 = q0 rear wheel, 2 = q1 neck, 3 = q2 head, 4 = q3 fork)
  %
  % Output:
  %  achievable    : 4x1 logical vector for the tasks [y_B; phi_D; z_F; z_D].
  %                  true if the task can still be controlled by the remaining
  %                  joints without disturbing a task of higher priority
  %                  (ignore that the final drink height is out of reach).

  % Implement your solution here...

  achievable = [];

end
