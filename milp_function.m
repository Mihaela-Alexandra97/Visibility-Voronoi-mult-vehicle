function [solution, fval, exitflag, buildTime, solveTime, totalTime,raport] = milp_min(numCities, numVehicles, travelTimes_vis, travelTimes_vor, e, l, M, eta,alpha,beta)
fprintf('MILP started\n')
tStart = tic;

prob = optimproblem('ObjectiveSense', 'minimize');
%% Decision variables
x = optimvar('x', numCities, numCities, numVehicles, ...
    'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);   % edge used?

z = optimvar('z', numCities, numCities, numVehicles, ...
    'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);   % visibility mode?

d = optimvar('d', numCities, numVehicles, 'LowerBound', -(10^6), 'UpperBound', 10^6);
% departure time from city i by vehicle k (can be negative for safety)

u = optimvar('u', numVehicles, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);  % vehicle active?

order = optimvar('order', numCities, numVehicles, ...
    'LowerBound', 0, 'UpperBound', numCities, 'Type', 'continuous');
q = optimvar('q',numCities,numCities,numVehicles,'Type','integer','LowerBound',0,'UpperBound',1); % x AND z
r = optimvar('r',numCities,numCities,numVehicles,'Type','integer','LowerBound',0,'UpperBound',1); % x AND NOT

%%
obj = 0;


raport=alpha/beta
% Priority 2: Minimize number of visibility edges used
for k = 1:numVehicles
    for i = 1:numCities
        for j = 1:numCities
            if i ~= j
                obj = obj +  alpha* z(i,j,k);
            end
        end
    end
end

% Priority 3: Softly minimize total travel time — ONLY on traversed edges
   % Increase to 10–50 if you want very short routes

for k = 1:numVehicles
    for i = 1:numCities
        for j = 1:numCities
            if i ~= j
                % Visibility-mode cost: only when x=1 AND z=1 → q(i,j,k)
                vis_cost = beta *  travelTimes_vis(i,j) * q(i,j,k);
                
                % Voronoi-mode cost: only when x=1 AND z=0 → r(i,j,k)
                vor_cost = beta * travelTimes_vor(i,j) * r(i,j,k);
                
                obj = obj + vis_cost + vor_cost;
            end
        end
    end
end

prob.Objective = obj;
%%

% q = x ∧ z
prob.Constraints.q1 = q <= x;
prob.Constraints.q2 = q <= z;
prob.Constraints.q3 = q >= x + z - 1;

% r = x ∧ (1−z)  ⇔  r = x - q
prob.Constraints.r_def = r == x - q;
% 2. z can only be 1 if edge is used: z <= x
for i = 1:numCities
    for j = 1:numCities
        if i ~= j
            for k = 1:numVehicles
                prob.Constraints.(['z_leq_x_' num2str(i) num2str(j) num2str(k)]) = z(i,j,k) <= x(i,j,k);
            end
        end
    end
end

% 3. Each customer visited exactly once
for i = 2:numCities
    visit_sum = 0;
    for j = 1:numCities
        if i ~= j
            for k = 1:numVehicles
                visit_sum = visit_sum + x(i,j,k);
            end
        end
    end
    prob.Constraints.(['visit_' num2str(i)]) = visit_sum == 1;
end

% 4. Flow conservation at customers
for i = 2:numCities
    for k = 1:numVehicles
        in_sum = 0;
        out_sum = 0;
        for j = 1:numCities
            if i ~= j
                in_sum  = in_sum  + x(j,i,k);
                out_sum = out_sum + x(i,j,k);
            end
        end
        prob.Constraints.(['flow_in_'  num2str(i) '_' num2str(k)]) = in_sum == out_sum;
        prob.Constraints.(['flow_out_' num2str(i) '_' num2str(k)]) = out_sum <= u(k);
    end
end

% 5. Depot: number of departures = number of returns = u(k)
for k = 1:numVehicles
    start_sum = 0;
    return_sum = 0;
    for j = 2:numCities
        start_sum   = start_sum   + x(1,j,k);
        return_sum  = return_sum  + x(j,1,k);
    end
    prob.Constraints.(['depot_start_'  num2str(k)]) = start_sum  == u(k);
    prob.Constraints.(['depot_return_' num2str(k)]) = return_sum == u(k);
end

% 6. Time windows
for i = 1:numCities
    for k = 1:numVehicles
        prob.Constraints.(['early_' num2str(i) '_' num2str(k)]) = d(i,k) >= e(i);
        prob.Constraints.(['late_'  num2str(i) '_' num2str(k)]) = d(i,k) <= l(i);
    end
end

% 7. Time propagation (fully linear)
for i = 1:numCities
    for j = 1:numCities
        if i ~= j
            for k = 1:numVehicles
                vis_t = travelTimes_vis(i,j);
                vor_t = travelTimes_vor(i,j);
                travel_t = z(i,j,k)*vis_t + (1 - z(i,j,k))*vor_t;

                if j == 1
                    % Going back to depot → only need to arrive before l(1)
                    prob.Constraints.(['time_to_depot_' num2str(i) '_' num2str(k)]) = ...
                        d(i,k) + travel_t <= l(1) + M*(1 - x(i,j,k));
                else
                    % Normal customer-to-customer or depot-to-customer
                    prob.Constraints.(['time_prop_' num2str(i) '_' num2str(j) '_' num2str(k)]) = ...
                        d(i,k) + travel_t <= d(j,k) + M*(1 - x(i,j,k));
                end
            end
        end
    end
end

% 8. Edge only used by active vehicles
for i = 1:numCities
    for j = 1:numCities
        if i ~= j
            for k = 1:numVehicles
                prob.Constraints.(['x_by_u_' num2str(i) num2str(j) num2str(k)]) = x(i,j,k) <= u(k);
            end
        end
    end
end
% 8b. No immediate backtracking: forbid x(i,j,k) + x(j,i,k) = 2
%     (i.e., robot k cannot go i→j and immediately j→i)
norev_constr = optimconstr(0);  % start with empty constraint array
idx = 1;

for i = 1:numCities
    for j = i+1:numCities          % only i < j to avoid double counting
        for k = 1:numVehicles
            norev_constr(idx) = x(i,j,k) + x(j,i,k) <= 1;
            idx = idx + 1;
        end
    end
end

prob.Constraints.noreverse = norev_constr;
% 9. Max vehicles (redundant but safe)
prob.Constraints.max_vehicles = sum(u) <= numVehicles;

% 10. MTZ subtour elimination
for k = 1:numVehicles
    prob.Constraints.(['order_depot_' num2str(k)]) = order(1,k) == 0;
end

for k = 1:numVehicles
    for i = 2:numCities
        for j = 2:numCities
            if i ~= j
                prob.Constraints.(['mtz_' num2str(i) '_' num2str(j) '_' num2str(k)]) = ...
                    order(i,k) - order(j,k) + numCities * x(i,j,k) <= numCities - 1;
            end
        end
    end
end

%% Solve
buildTime = toc(tStart);
fprintf('Model built in %.3f seconds\n', buildTime);

tStart = tic;
options = optimoptions('intlinprog', 'Display', 'off');

[solution, fval, exitflag, output] = solve(prob, 'Options', options);

solveTime = toc(tStart);
totalTime = buildTime + solveTime;
fprintf('milp ended T2T3\n');
 disp(datetime('now'));   
fprintf('MILP ended\n')
end