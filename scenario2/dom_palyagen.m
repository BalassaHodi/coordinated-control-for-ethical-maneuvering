% SCENARIO 2

%{
This function implements the path-planning layer for the dominant vehicle in the cooperative control design.
The path-planning is created with RRT* algorithm on the created vehicleCostmap.
---
The input:
- a vehstate array: [1-by-m double] (m: states of both vehicles: [dom_x, dom_y dom_psi, sub_x, sub_y, sub_psi])
The output:
- a palya array: [n-by-m array] (n: time steps) (m: the poses of the reference route from the RRT: [x, y, psi, length])
%}

function kimenet = dom_palyagen(input)

% Global variables
global pedestrian;
global dom_goal_pos;
global OK;


% Create the costmap
mapWidth = 30;
mapLength = 10;
costVal = 0;
cellSize = 0.5;
costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize);


% Create the inflated area of the obstacles based on the dimensions of the dominant vehicle
vehicleDims = vehicleDimensions(4.5,1.7);   % 4.5 m long, 1.7 m wide
numCircles = 3;
ccConfig = inflationCollisionChecker(vehicleDims,numCircles);
costmap.CollisionChecker = ccConfig;


% Create the obstacles
% The pedestrian in the middle of the road
occupiedVal = 1;
xyPoint1 = [10,2.5];
setCosts(costmap,xyPoint1,occupiedVal);
pedestrian = xyPoint1;

% The subordinate vehicle in the other lane
xyPoint2 = [input(4)-2*cos(input(6)), input(5)-2*sin(input(6)); input(4), input(5); input(4)+2*cos(input(6)), input(5)+2*sin(input(6))];
setCosts(costmap,xyPoint2,occupiedVal);

% The sides of the roads
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' 0*ones(31,1)];
setCosts(costmap,xyPoint3,occupiedVal);
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' mapLength*ones(31,1)];
setCosts(costmap,xyPoint3,occupiedVal);

% Plot the costmap for debugging
% figure;
% plot(costmap);



% Set the startPose and the goalPose of the dominant vehicle
startPose = [input(1), input(2), input(3)*180/pi];
goalPose = dom_goal_pos;


% Check the startpose
OK = checkFree(costmap,startPose);