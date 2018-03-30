% ERROR_NONLINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the total error of all measurements (odometry and landmark)
% given the current state estimate
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     err     - total error of all measurements
%
function err = error_nonlinear(x, odom, obs, sigma_odom, sigma_landmark)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);    % landmark measurement dimension

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize error
err = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bp = zeros(2,1);

bo = zeros(numel(odom),1);
for i = 1:size(odom, 1)
    curr_bo = odom(i,:)' - meas_odom(x(i*2-1), x(i*2), x(i*2+1), x(i*2+2));
    bo(i*2-1:i*2) = sqrt(inv(sigma_odom))*curr_bo;
end

bl = zeros(n_obs*2, 1);
for i = 1:size(obs, 1)
    r = obs(i,1);
    l = obs(i,2);
    l_pos = [ obs(i,3);
              obs(i,4) ];
    r_est = [ x(r*2-1); 
              x(r*2) ];
    l_est = [ x(l*2+n_poses+1); 
              x(l*2+n_poses+2) ];
    
    curr_bl = l_pos - meas_landmark(r_est(1), r_est(2), l_est(1), l_est(2));
    curr_bl(1) = wrapToPi(curr_bl(1));
    bl(i*2-1:i*2) = sqrt(inv(sigma_landmark))*curr_bl;
end

b = vertcat(bp, bo, bl);

[err, ~] = sumsqr(b);

end
