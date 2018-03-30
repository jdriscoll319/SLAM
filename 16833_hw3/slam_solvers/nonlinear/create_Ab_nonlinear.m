% CREATE_AB_NONLINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the A and b matrices for the 2D nonlinear SLAM problem
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
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize matrices
%A = zeros(M, N);
%b = zeros(M, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ho = [-1 0 1 0;
      0 -1 0 1];
Ho = sqrt(inv(sigma_o))*Ho;


Ap = horzcat(eye(2), zeros(2, N-2));
bp = zeros(2,1);

Ao = zeros(n_odom*2, N);
bo = zeros(numel(odom),1);
for i = 1:size(odom, 1)
    Ao(i*2-1:i*2, i*2-1:i*2+2) = Ho;
    curr_bo = odom(i,:)' - meas_odom(x(i*2-1), x(i*2), x(i*2+1), x(i*2+2));
    bo(i*2-1:i*2) = sqrt(inv(sigma_o))*curr_bo;
end
%A(3:numel(odom)+2, 1:numel(odom)+4) = Ao;

Al = zeros(n_obs*2, N);
bl = zeros(n_obs*2, 1);
lm_offset = n_odom*2+2;
for i = 1:size(obs, 1)
    r = obs(i,1);
    l = obs(i,2);
    l_pos = [ obs(i,3);
              obs(i,4) ];
    r_est = [ x(r); 
             x(r+1) ];
    l_est = [ x(l*2+n_poses+1); 
              x(l*2+n_poses+2) ];
    
    Hm = meas_landmark_jacobian(r_est(1), r_est(2), l_est(1), l_est(2));
    Hm = sqrt(inv(sigma_l))*Hm;
    %Robot part of Hm
    Hm_r = Hm(1:2, 1:2);
    %Landmark part of Hm
    Hm_l = Hm(1:2, 3:4);
    
    %insert robot part of Hm
    Al(i*2-1:i*2, r*2-1:r*2) = Hm_r;
    
    %insert landmark part of Hm
    Al(i*2-1:i*2, l*2-1+lm_offset:l*2+lm_offset) = Hm_l;
    
    curr_bl = l_pos - meas_landmark(r_est(1), r_est(2), l_est(1), l_est(2));
    bl(i*2-1:i*2) = sqrt(inv(sigma_l))*curr_bl;
end

%A(numel(odom)+3:end, 1:end) = Al;
A = vertcat(Ap, Ao, Al);
b = vertcat(bp, bo, bl);


%% Make A a sparse matrix 
As = sparse(A);
