% function [path_indices,path_coord,path_cost,path_timing] = path_in_full_dual(path_red,Edge_to_path1,Edge_to_path2,Nodes_coord1,Nodes_coord2,colors,ok,vs)
% 
% path_coord=[];
% if ok==1
%     path_indices=[];
%    path = Edge_to_path2{path_red(1),path_red(2)}; 
%    path_indices = [path_indices, path];
%    for i=1:length(path_indices)
%         path_coord(1,i) = Nodes_coord2(1,path_indices(i));
%         path_coord(2,i) = Nodes_coord2(2,path_indices(i)); 
%    end
% else
%    path_indices=[];
%    path = Edge_to_path1{path_red(1),path_red(2)}; 
%    path_indices = [path_indices, path];
%    for i=1:length(path_indices)
%         path_coord(1,i) = Nodes_coord1(1,path_indices(i));
%         path_coord(2,i) = Nodes_coord1(2,path_indices(i)); 
%    end 
% end
% path_cost=0;
% path_timing=0;
% for i=1:size(path_coord,2)-1
%     path_cost = path_cost + norm(path_coord(:,i)-path_coord(:,i+1));    %distance to travel
%     path_timing=path_timing+path_cost/vs;
%     %just for visualizing the path in visibility graph
%     if ok==1
%          plot(path_coord(1,[i,i+1]),path_coord(2,[i,i+1]),'Color',colors,'LineWidth',2,'LineStyle',':');
%          pause(0.05);
% 
%     else
%     plot(path_coord(1,[i,i+1]),path_coord(2,[i,i+1]),'Color',colors,'LineWidth',2);
%     pause(0.05);
% end
% end
% function [path_indices, path_coord, path_cost, path_timing] = path_in_full_dual(path_red, Edge_to_path1, Edge_to_path2, Nodes_coord1, Nodes_coord2, colors, ok, vs)
% % PATH_IN_FULL_DUAL - Plot the full path (visibility or Voronoi) with direction arrows
% % Inputs:
% %   path_red        - reduced path (pair of nodes)
% %   Edge_to_path1   - mapping for Voronoi edges
% %   Edge_to_path2   - mapping for Visibility edges
% %   Nodes_coord1/2  - node coordinates for Voronoi/Visibility
% %   colors          - color for path
% %   ok              - 1 for Visibility, 0 for Voronoi
% %   vs              - vehicle speed
% %
% % Outputs:
% %   path_indices    - sequence of node indices in full graph
% %   path_coord      - coordinates of path
% %   path_cost       - total path distance
% %   path_timing     - travel time based on vs
% 
% path_coord = [];
% path_indices = [];
% 
% % --- Select appropriate path and coordinates ---
% if ok == 1
%     % Visibility graph
%     path = Edge_to_path2{path_red(1), path_red(2)};
%     path_indices = [path_indices, path];
%     for i = 1:length(path_indices)
%         path_coord(1, i) = Nodes_coord2(1, path_indices(i));
%         path_coord(2, i) = Nodes_coord2(2, path_indices(i));
%     end
% else
%     % Voronoi graph
%     path = Edge_to_path1{path_red(1), path_red(2)};
%     path_indices = [path_indices, path];
%     for i = 1:length(path_indices)
%         path_coord(1, i) = Nodes_coord1(1, path_indices(i));
%         path_coord(2, i) = Nodes_coord1(2, path_indices(i));
%     end
% end
% 
% % --- Compute path cost and timing ---
% path_cost = 0;
% path_timing = 0;
% 
% % --- Plot path with arrows ---
% for i = 1:size(path_coord, 2) - 1
%     x_start = path_coord(1, i);
%     y_start = path_coord(2, i);
%     dx = path_coord(1, i + 1) - x_start;
%     dy = path_coord(2, i + 1) - y_start;
% 
%     % Update cost and timing
%     segment_length = norm([dx, dy]);
%     path_cost = path_cost + segment_length;
%     path_timing = path_timing + segment_length / vs;
% 
%     % Plot with arrow
%     if ok == 1
%         % Visibility graph (dashed line)
%         quiver(x_start, y_start, dx, dy, 0, ...
%             'Color', colors, 'LineWidth', 2, ...
%             'LineStyle', ':', 'MaxHeadSize', 1);
%     else
%         % Voronoi graph (solid line)
%         quiver(x_start, y_start, dx, dy, 0, ...
%             'Color', colors, 'LineWidth', 2, ...
%             'LineStyle', '-', 'MaxHeadSize', 1);
%     end
% 
%     pause(0.05);
% end
% 
% end
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
% This version respects the current figure (gcf) — no more plotting on wrong windows!

path_coord = [];
path_indices = [];

% === CRITICAL FIX: Make sure we draw on the correct (current) figure ===
              % Keep existing content
 % Visibility: dashed, slightly thicker
        figure(gcf);           % Force MATLAB to target the figure that called us
        hold on; 
% --- Select appropriate path and coordinates ---
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

% --- Compute cost and timing ---
path_cost = 0;
path_timing = 0;

% --- Plot segment by segment with arrows ---
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
    
    % Plot arrow (quiver is perfect for direction)
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

% Optional: small pause so you can see animation (remove if not wanted)
 % pause(0.03);

end