function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    map_points = fusion_map.pointcloud.Location;
    map_colors = fusion_map.pointcloud.Color;
    map_normals = fusion_map.normals;
    map_ccounts = fusion_map.ccounts;
    map_times = fusion_map.times;
    
    updated_points = reshape( updated_map.points, [], 3);
    updated_colors = reshape( updated_map.colors, [], 3);
    updated_normals = reshape( updated_map.normals, [], 3);
    updated_ccounts = reshape( updated_map.ccounts, [], 1);
    updated_times = reshape( updated_map.times, [], 1);
    
    map_points(proj_flag,:) = [];
    map_points = [map_points; updated_points];
    
    map_colors(proj_flag,:) = [];
    map_colors = [map_colors; updated_colors];
    
    map_normals(proj_flag,:) = [];
    map_normals = [map_normals; updated_normals];
    
    map_ccounts(proj_flag) = [];
    map_ccounts = [map_ccounts; updated_ccounts];
    
    map_times(proj_flag) = [];
    map_times = [map_times; updated_times];
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   