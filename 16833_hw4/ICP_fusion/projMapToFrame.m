function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera)====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    %%
    %Initialization and setup
    K = [fx 0  cx;
         0  fy cy;
         0  0  1 ];
    T = inv(tform.T');
    t = T(1:3,end);
    R = T(1:3, 1:3);
    P = K*[R t];
    V = [fusion_map.pointcloud.Location'; ones(1, size(fusion_map.pointcloud.Location, 1))];
    color = fusion_map.pointcloud.Color';
    N = fusion_map.normals';
    c = fusion_map.ccounts;
    time = fusion_map.times;
    
    %%
    %First just transform the points to camera frame and find the negative
    %Z's
    cam_points = T*V;
    cam_points = cam_points ./ cam_points(4,:);
    neg_z = any(cam_points(3,:) < 0, 1);  %%Index of negative z's
    
    %%
    %Project points onto camera frame and find points outside the frame
    to_cam_plane = P*V;
    %normalize that shit
    to_cam_plane = round( to_cam_plane./to_cam_plane(3,:) );
    
    outside_frame = any(to_cam_plane(1,:) <= 0 | ...
                        to_cam_plane(2,:) <= 0 | ...
                        to_cam_plane(1,:) > w  | ...
                        to_cam_plane(2,:) > h, 1);

    %%
    %Dump all of the bad indices in everything to zero
    bad_indices = logical(neg_z + outside_frame);
    V(:,bad_indices) = 0;
    proj_flag = (~bad_indices)';
    
    %%
    %convert to 3D matrix
    proj_points = zeros(h,w,3);
    proj_colors = zeros(h,w,3);
    proj_normals = zeros(h,w,3);
    proj_ccounts = zeros(h,w);
    proj_times = zeros(h,w);
    
    for i = 1:numel(bad_indices)
        if ~bad_indices(i)
            x = to_cam_plane(1,i);
            y = to_cam_plane(2,i);
            
            proj_points(y, x, :) = V(1:3, i);
            
            proj_colors(y, x, :) = color(:, i);
            
            proj_normals(y, x, :) = N(:, i);
            
            proj_ccounts(y, x) = c(i);
            proj_times(y,x) = time(i);
        end
    end
    
 
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
