%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

%==== TEST: Setup uncertianty parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.08;
sig_r = 0.01;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_r2, sig_beta2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====
k = 6;
% Write your code here...
landmark = zeros(2*k,1);
for i = 1:2:numel(measure)
    beta = measure(i);
    r = measure(i+1);

    landmark(i) = pose(1) + r * cos(beta + pose(3));
    landmark(i+1) = pose(2) + r * sin(beta + pose(3));
end

landmark_cov = diag(ones(1, 2*k)*0.02);

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ===
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    
    control_update = [ d*cos(x(3)); ...
                       d*sin(x(3)); ...
                       alpha];
    Fx = [eye(3) zeros(3, 2*k)];
    x_pre = x + (Fx' * control_update);
    
    G = [1 0 -d*sin(x(3)); ...
         0 1 d*cos(x(3));  ...
         0 0 1];
    G_big = [G zeros(3, 2*k); ...
             zeros(2*k, 3) eye(2*k)];
    G_big_T = [G' zeros(3, 2*k); ...
               zeros(2*k, 3) eye(2*k)];
    
    GPG = G_big * P * G_big';

    Rt_big = Fx' * control_cov * Fx;
    
    P_pre = GPG + Rt_big;
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    for i = 1:2:numel(measure)
        r = measure(i+1);
        phi = measure(1);
        
        z = [r; ...
             phi];
        
        delta = [x_pre(i+3) - x_pre(1); ...
                 x_pre(i+4) - x_pre(2)];
        
        q = delta' * delta;
        
        z_est = [sqrt(q); ...
                 atan2(delta(2), delta(1)) - x_pre(3)];
        %{
        lm_count = ceil(i/2);
        col13 = [eye(3); zeros(2,3)];
        colobs = zeros(5, 2*lm_count - 2);
        collm = [zeros(3, 2); eye(2)];
        collast = zeros(5, 2*k - 2*lm_count);
        F = [col13 colobs collm collast];
        
        H_low = (1/q) * [-sqrt(q)*delta(1) -sqrt(q)*delta(2) 0 sqrt(q)*delta(1) sqrt(q)*delta(2); ...
                         delta(2) -delta(1) -q -delta(2) delta(1)] ;
        H = H_low * F;
        %}
        lm_count = ceil(i/2);

        colobs = zeros(2, 2*lm_count - 2);
        collm =  eye(2);
        collast = zeros(2, 2*k - 2*lm_count);
        F = [colobs collm collast];
        
        Hp = (1/q) * [-sqrt(q)*delta(1) -sqrt(q)*delta(2) 0; ...
                      delta(2) -delta(1) -q ];
        Hl = (1/q) * [sqrt(q)*delta(1) sqrt(q)*delta(2); ...
                      -delta(2) delta(1)];
        Hl = Hl * F;
        
        H = [Hp Hl];
        %}
        K = P_pre * H'  * ( H * P_pre * H' + measure_cov)^(-1);
        
        x_pre = x_pre + K * (z - z_est);
        KH = K*H;
        P_pre = (eye(size(KH)) - KH) * P_pre;
        
    end
        
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
gt = [3 6 3 12 7 8 7 14 11 6 11 12];
for i = 1:2:numel(gt)
    plot(gt(i), gt(i+1), 'x', 'Color', 'red');
end

%==== Close data file ====
fclose(fid);
