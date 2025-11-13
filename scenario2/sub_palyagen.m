% SCENARIO 2

%{
This function implements the path-planning layer for the subordinate vehicle in the cooperative control design.
The path-planning is created with RRT* algorithm on the created vehicleCostmap.
---
The input:
- a vehstate array: [1-by-m double] (m: states of both vehicles: [dom_x, dom_y dom_psi, sub_x, sub_y, sub_psi])
The output:
- a palya array: [n-by-m array] (n: time steps) (m: the poses of the reference route from the RRT: [x, y, psi, length])
%}

function kimenet = sub_palyagen(input,other_palya)

% Global variables
global pedestrian;
global sub_goal_pos;
global sub_OK;
global sub_all_palya;
global sub_palya;
global sub_warnings;
global sub_emergency;
global t;
global sub_costmap;
global danger;


% Clear the palya from the previous iteration
sub_palya = double.empty();


% If there is no danger, then the palya shall be a straight line in the middle of the lane
if ~danger
    sub_palya(1,:) = [input(4), input(5), input(6), 0];
    idx = 2;
    l = 2;
    while sub_palya(end,1) >= sub_goal_pos(1)
        sub_palya(idx,:) = [sub_palya(idx-1,1)-l, 7.5, 0, sub_palya(idx-1,4)+l];
        idx = idx + 1;
    end
    kimenet = sub_palya;
    % disp('Nincs veszély.');
    return
end


% Create the costmap
mapWidth = 30;
mapLength = 10;
costVal = 0;
cellSize = 0.5;
sub_costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize);


% Create the inflated area of the obstacles based on the dimensions of the subordinate vehicle
vehicleDims = vehicleDimensions(4.5,1.7);
numCircles = 3;
ccConfig = inflationCollisionChecker(vehicleDims,numCircles);
sub_costmap.CollisionChecker = ccConfig;


% Create the obstacles
% The pedestrian is in the middle of the other lane
occupiedVal = 1;
xyPoint1 = pedestrian;
setCosts(sub_costmap,xyPoint1,occupiedVal);

% The position of the dominant vehicle on the other lane
xyPoint2 = [input(1)-2*cos(input(3)), input(2)-2*sin(input(3)); input(1), input(2); input(1)+2*cos(input(3)), input(2)+2*sin(input(3))];
setCosts(sub_costmap,xyPoint2,occupiedVal);

% The sides of the roads
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' 0*ones(31,1)];
setCosts(sub_costmap,xyPoint3,occupiedVal);
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' mapLength*ones(31,1)];
setCosts(sub_costmap,xyPoint3,occupiedVal);

% The 4 ahead points of the reference path of the dominant vehicle
if size(other_palya,1) >= 5 
    occupiedVal = 1;
    xyPoint4 = [other_palya(2,1), other_palya(2,2); other_palya(3,1), other_palya(3,2); other_palya(4,1), other_palya(4,2); other_palya(5,1), other_palya(5,2)];
    setCosts(sub_costmap,xyPoint4,occupiedVal);
end

% Plot the costmap for debugging
% figure;
% plot(sub_costmap);



% Check the startPose
