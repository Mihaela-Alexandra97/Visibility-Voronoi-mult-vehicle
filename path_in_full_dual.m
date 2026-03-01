function [path_indices, path_coord, path_cost, path_timing] = path_in_full_dual(...
    path_red, Edge_to_path1, Edge_to_path2, Nodes_coord1, Nodes_coord2, colors, ok, vs)
% PATH_IN_FULL_DUAL - Plot the full path (visibility or Voronoi) with direction arrows
% Inputs:
%   path_red      - [i j] reduced edge
%   Edge_to_path1 - Voronoi edge → full node sequence
%   Edge_to_path2 - Visibility edge → full node sequence
%   Nodes_coord1  - Voronoi node coordinates
%   Nodes_coord2  - Visibility node coordinates
%   colors        - RGB color for this vehicle
%   ok            - 1 = use Visibility, 0 = use Voronoi
%   vs            - vehicle speed
%


path_coord = [];
path_indices = [];


              % Keep existing content
 % Visibility: dashed, slightly thicker
        figure(gcf);           
        hold on; 

if ok == 1
    % Visibility graph
    path = Edge_to_path2{path_red(1), path_red(2)};
    nodes = Nodes_coord2;
else
    % Voronoi graph
    path = Edge_to_path1{path_red(1), path_red(2)};
    nodes = Nodes_coord1;
end

path_indices = path;

% Build coordinate matrix
path_coord = nodes(:, path);  % 2 x length


path_cost = 0;
path_timing = 0;

for seg = 1:(length(path)-1)
    i = path(seg);
    j = path(seg+1);
    
    x_start = nodes(1, i);
    y_start = nodes(2, i);
    dx = nodes(1, j) - x_start;
    dy = nodes(2, j) - y_start;
    
    segment_length = norm([dx, dy]);
    path_cost = path_cost + segment_length;
    path_timing = path_timing + segment_length / vs;
    
    if ok == 1
       
        quiver(x_start, y_start, dx, dy, 0, ...
            'Color', colors, 'LineWidth', 2.5, 'LineStyle', ':', ...
            'MaxHeadSize', 1.2, 'AutoScale', 'off');
    else
 
        quiver(x_start, y_start, dx, dy, 0, ...
            'Color', colors, 'LineWidth', 2, 'LineStyle', '-', ...
            'MaxHeadSize', 1.0, 'AutoScale', 'off');
    end
end

 % pause(0.03);

end


