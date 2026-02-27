function [env_bounds,numCities,numVehicles,numObstacles,vehicleSpeed,M,e,l] = define_data()

numCities     = 7;
numVehicles   = 10;
numObstacles  = 5;
vehicleSpeed  = 10;
M             = 1000;
env_bounds    = [0 200 0 200];


travelTimeRow = zeros(1, numCities);


travelTimeRow(2:end) = 5 + 10*rand(1, numCities-1);   % [5,15] seconds

e = zeros(numCities,1);
l = zeros(numCities,1);

for i = 2:numCities
    e(i) = ceil(travelTimeRow(i)) + randi(4);
    l(i) = e(i) + randi([10, 20]);
end

e(1) = 0;      % depot
l(1) = 1e6;    % no upper bound

end

