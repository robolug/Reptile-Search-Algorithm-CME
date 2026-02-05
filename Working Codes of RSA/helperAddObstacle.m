% Add obstacles to the map at specific locations. Inputs to
% helperAddObstacle are obstacleWidth, obstacleHeight and obstacleLocation.


%helperAddObstacle Adds obstacles to the occupancy map
function helperAddObstacle(map,obstacleWidth,obstacleHeight,obstacleLocation)
values = ones(obstacleHeight,obstacleWidth);
setOccupancy(map,obstacleLocation,values)
end