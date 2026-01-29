function [env_bounds,numCities,numVehicles,numObstacles,vehicleSpeed,M,e,l]=define_data();

env_bounds=[0 200 0 200];

travelTimeRow = [0, 12.50, 10.69, 6.25, 11.26, 8.28, 11.27, 10.86, 10.01, 5.34, ...
                  13.22, 7.88, 9.40, 14.05, 6.98, 12.11, 11.75, 10.56, 9.87, 8.44];
numCities = 7;
e = zeros(numCities,1);
l = zeros(numCities,1);

for i = 2:numCities
    e(i) = ceil(travelTimeRow(i)) + randi(4);   % e.g., arrival time from depot + buffer
    l(i) = e(i) + randi([10, 20]);
;                      
end

e(1) = 0;    % depot
l(1) = 1e6;  % no upper bound (or Inf)
numVehicles=10;
numObstacles=5;
vehicleSpeed=10;
M=1000;

end