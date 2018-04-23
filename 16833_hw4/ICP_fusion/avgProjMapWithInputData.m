function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====
    %%
    %make 3d matrices so math works
    counts_3d = repmat(proj_ccounts, [1 1 3]);
    is_use_3d = repmat(is_use, [1 1 3]);
    alpha_3d = repmat(alpha, [1 1 3]);
    proj_colors = double(proj_colors);
    input_colors = double(input_colors);

    
    %%
    %Do the averaging now
    updated_points = proj_points;
    updated_points(is_use_3d) = ...
        (counts_3d(is_use_3d).*proj_points(is_use_3d) + alpha_3d(is_use_3d).*input_points(is_use_3d)) ...
        ./ (counts_3d(is_use_3d) + alpha_3d(is_use_3d));
    
    updated_colors = proj_colors;
    updated_colors(is_use_3d) = ...
        (counts_3d(is_use_3d).*proj_colors(is_use_3d) + alpha_3d(is_use_3d).*input_colors(is_use_3d)) ...
        ./ (counts_3d(is_use_3d) + alpha_3d(is_use_3d));
%     updated_colors = input_colors;
    
    updated_normals = proj_normals;
    updated_normals(is_use_3d) = ...
        (counts_3d(is_use_3d).*proj_normals(is_use_3d) + alpha_3d(is_use_3d).*input_normals(is_use_3d)) ...
        ./ (counts_3d(is_use_3d) + alpha_3d(is_use_3d));
    
    updated_ccounts = proj_ccounts;
    updated_ccounts(is_use) = proj_ccounts(is_use) + alpha(is_use);
    
    updated_times = proj_times;
    updated_times(is_use) = t;%proj_times(is_use) + t;
        
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end