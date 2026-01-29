clc
clearvars
close all

%  Define environment and base data 
[env_bounds,numCities,numVehicles,numObstacles,vehicleSpeed,M,e,l] = define_data();

[Obstacles, Cities, fig_env] = define_obstacles_cities(env_bounds, numObstacles, numCities, e, l);
%%
[obstacles]=extract_obstacles(env_bounds, numObstacles, fig_env, Obstacles);
%%




%  Create separate figure handles 
fig_vis = figure('Name','Visibility Graph');  % visibility graph
fig_vor = figure('Name','Voronoi Graph');     % voronoi graph

%  Reuse the environment base in both 
copyobj(allchild(fig_env), fig_vis);          % copy environment to fig_vis
copyobj(allchild(fig_env), fig_vor);          % copy environment to fig_vor

% Construct graphs and plot on their figures 
[Nodes_vis, Adj_vis, time_vis] = construct_visibility_graph(Obstacles, Cities, fig_vis, 1);
[GVD_Vert, GVD_Adj, Adj_vor, Nodes_vor, totalTimeVoronoi] = GVD_construct(env_bounds, obstacles, Cities, fig_vor, 1);
Nodes_vor = Nodes_vor';

%  Graph reduction 
[distanceMatrix_vor, Edge_to_path_vor, ~, travelTimes_vor] = reduce_graph(Adj_vor, 1:numCities, vehicleSpeed);
[distanceMatrix_vis, Edge_to_path_vis, ~, travelTimes_vis] = reduce_graph(Adj_vis, 1:numCities, vehicleSpeed);
%%

% Create combined graph figure with faint visibility & voronoi

fig_combined = figure('Name', 'Combined Graphs (Environment + Vis + Vor)');

% Copy environment (cities + obstacles)
copyobj(allchild(fig_env), fig_combined);
hold on;

% Add faint visibility graph (plotFlag = 0)
[~, ~, ~] = construct_visibility_graph(Obstacles, Cities, fig_combined, 0);

% Add faint Voronoi graph (plotFlag = 0)
[~, ~, ~, ~] = GVD_construct(env_bounds, obstacles, Cities, fig_combined, 0);
D=pdist2(Cities',Cities');

%%
% --- MILP call ---


disp(datetime('now'));       
alpha=250;
beta=10;
[solution, fval, exitflag, buildTime, solveTime, totalTime,raport]= milp_function(numCities, numVehicles, travelTimes_vis, travelTimes_vor, e, l, M, vehicleSpeed,alpha,beta);

% --- Plot robot paths separately ---
colors = lines(numVehicles);
if exitflag ~= 1
    disp('MILP ERROR');
else
    for k = 1:numVehicles
    
    % Skip if this robot is NOT used in the solution
    if round(solution.u(k)) == 0
        continue;
    end
    
    fig_robot = figure('Name', ['Robot ' num2str(k)]);
    figr{k} = fig_robot;
    copyobj(allchild(fig_combined), fig_robot);   % copy base environment
    
    x_ij = round(solution.x(:,:,k));
    z_ij = round(solution.z(:,:,k));
    
    i = 1;
    no_trans = sum(sum(x_ij));
    
    for trans = 1:no_trans
        j = find(x_ij(i,:) == 1);
        if isempty(j), break; end
        
        if z_ij(i,j) == 0
            [~, ~, ~, ~] = path_in_full_dual([i,j], Edge_to_path_vor, Edge_to_path_vis, ...
                                            Nodes_vor, Nodes_vis, colors(k,:), 0, vehicleSpeed);
        else
            [~, ~, ~, ~] = path_in_full_dual([i,j], Edge_to_path_vor, Edge_to_path_vis, ...
                                            Nodes_vor, Nodes_vis, colors(k,:), 1, vehicleSpeed);
        end
        
        i = j;
    end
end
end
% Get indices of active vehicles
activeVehicles = find(solution.u);  % e.g., [6, 8]

% Initialize a cell array to store clean arrival times per vehicle
arrivalTimes = cell(1, numel(activeVehicles));

% For each active vehicle, collect visited nodes and their arrival times
for idx = 1:numel(activeVehicles)
    k = activeVehicles(idx);   % actual vehicle index (e.g., 6 or 8)

    visitedNodes = any(solution.x(:,:,k), 1);   % node j is visited if any i→j exists
    visitedNodes(1) = 0;  % optional: exclude depot if needed

    % Extract arrival times only at visited nodes
    times = solution.d(:,k);
    arrivalTimes{idx} = [find(visitedNodes(:)), times(visitedNodes)];  % [node, time]
end
for idx = 1:numel(activeVehicles)
    k = activeVehicles(idx);   % vehicle index (e.g., 6 or 8)

    % Determine which nodes vehicle k visits
    visitedNodes = any(solution.x(:,:,k), 1);   % j is visited if any i → j exists
    visitedNodes(1) = 0;  % optionally exclude depot (node 1)

    % Extract relevant data
    nodeIndices = find(visitedNodes(:));
    arrivalTimes = solution.d(nodeIndices, k);
    earliestTimes = e(nodeIndices);
    latestTimes = l(nodeIndices);

    % Combine into a table
    T = table(nodeIndices, arrivalTimes, earliestTimes, latestTimes, ...
        'VariableNames', {'Node', 'ArrivalTime', 'WindowStart', 'WindowEnd'});

    % Display
    fprintf('Vehicle %d:\n', k);
    disp(T);
end
%%
% %%  
% ACO parameters
params.alpha = 1;
params.beta = 4;
params.rho = 0.1;
params.Q = 1;
params.nAnts = 10;
params.nIter = 50;
params.timePenalty = 100;     % penalty for time window violations
params.robotPenalty = 10;   % per robot used
params.visPenalty = 100;      % penalty for using visibility graph (riskier)
params.vorPenalty = 10;        % no penalty for Voronoi

% Run ACO
 [bestSolution, bestCost, numVisArcs, numVorArcs, arrivalTimes, robotModeMatrix,compTimeACO] = ACO_VRPTW_vv(numCities, numVehicles, distanceMatrix_vis, travelTimes_vis, travelTimes_vor, e, l, params);


colors = lines(numVehicles);

for kv = 1:numVehicles
  
    route = bestSolution{kv};   % route for vehicle kv

    if length(route) > 2
           fig_robot_ACO = figure('Name', ['ACO Robot ' num2str(kv)]);
       copyobj(allchild(fig_combined),  fig_robot_ACO ); 

        % Iterate through each arc in the route
        for idx = 1:(length(route)-1)
            i = route(idx);
            j = route(idx+1);

            % Get mode from robotModeMatrix: 1=visibility, 2=voronoi
            modeUsed = robotModeMatrix(i,j,kv);

            % Plot the appropriate path based on modeUsed
            if modeUsed == 1  % Visibility graph
                [path_indices, path_coord, path_cost] = path_in_full_dual( ...
                    [i,j], Edge_to_path_vor, Edge_to_path_vis, ...
                    Nodes_vor, Nodes_vis, colors(kv,:), 1, vehicleSpeed);

            elseif modeUsed == 2  % Voronoi graph
                [path_indices, path_coord, path_cost] = path_in_full_dual( ...
                    [i,j], Edge_to_path_vor, Edge_to_path_vis, ...
                    Nodes_vor, Nodes_vis, colors(kv,:), 0, vehicleSpeed);
            else
                % modeUsed == 0 means arc unused – skip
                continue;
            end

            clear path_indices
        end
    else
        % Vehicle didn't use a route (or trivial route), skip
        continue
    end
    figr_ACO{kv}= fig_robot_ACO;
end

