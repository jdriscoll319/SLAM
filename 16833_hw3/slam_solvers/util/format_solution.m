% FORMAT_SOLUTION
% 16-831 Fall 2016 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
%     x           - State vector that contains all poses and landmarks
%                   (you decide the order of variables - make it easy!)
%     n_poses     - Number of poses in the state vector
%     n_landmarks - Number of landmarks in the state vector
%     p_dim       - Dimension of the pose (2 in this case)
%     l_dim       - Dimension of the landmark (2 in this case)
%
% Returns:
%     traj        - n_poses x p_dim matrix with the x and y coordinates in
%                   columns 1 and 2, respectively
%     landmarks   - n_landmarks x l_dim matrix with the x and y coordinates 
%                   in columns 1 and 2, respectively
%
function [traj, landmarks] = format_solution(x, n_poses, n_landmarks, p_dim, l_dim)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traj = reshape(x(1:n_poses*2), 2,[])';
landmarks = reshape(x(n_poses*2+1:end), 2, [])';

end