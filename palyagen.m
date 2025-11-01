%{
This function implements the path-planning layer of the coordinated control.
The path-planning is created with RRT* algorithm on the created vehicleCostmap.
---
The input:
- a vehstate vector: [1-by-m double] (m: states of av. and hv.: [x_av, y_av, psi_av, x_hv] for one time step)
The output:
- a palya matrix: [n-by-m array] (n: time steps) (m: the poses of the reference route from the RRT: [x, y, psi, length])
%}

function kimenet = palyagen(input)

% Global variables
global OK;
global costmap;
global palya;


% Create the costmap
mapWidth = 30;
mapLength = 10;
costVal = 0;
cellSize = 0.5;
costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize);


% Create the inflated area of the obstacles based on the dimension of the av.
vehicleDims = vehicleDimensions(4.5,1.7);   % 4.5 m long, 1.7 m wide
numCircles = 3;
ccConfig = inflationCollisionChecker(vehicleDims,numCircles);
costmap.CollisionChecker = ccConfig;


% Create the obstacles
% The pedestrian in the middle of the road
occupiedVal = 1;
xyPoint1 = [10,2.5]; 
setCosts(costmap,xyPoint1,occupiedVal);

% The hv. in the other lane based on its current x_hv position from the input
% The hv. is created by inflating 3 points
xyPoint2 = [input(4),7.5; input(4)+2.25,7.5; input(4)+4.5,7.5]; 
setCosts(costmap,xyPoint2,occupiedVal);

% The sides of the roads
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' 0*ones(31,1)];
setCosts(costmap,xyPoint3,occupiedVal);
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' mapLength*ones(31,1)];
setCosts(costmap,xyPoint3,occupiedVal);

% Plot the costmap for debugging
% plot(costmap)


% Set the startPose and the goalPose of the av.
% The startPose is based on the input, while the goalPose is the end of the av. lane
startPose = [input(1), input(2), input(3)*180/pi]; % HERE WAS A BUG, the psi was not converted into degrees, while the startPose and the goalPose has to be in degrees!
goalPose  = [25, 2.5, 0];


% Check the startpose
OK = checkFree(costmap, startPose);
if ~OK
    kimenet = palya;
    disp('A startPose nem megfelelő');
    return
end

%{
ERROR HERE, that has to be fixed!
Here is a problem. The path planning algorithm cannot create a path from
the 3rd iteration in the loop of Simulation.m. It is beacuse it cannot
create a proper path without collision. To fix this we have to create the
environment a little different (for example less inflated area ...).
This has to be done, because now the program is not working well.

This ERROR was only present after I started refactor the code, because
there was a mistake, that the startPose should've receive degrees data, but
it received radians instead.

One approach to make it work: only refresh the path if the previous path
has a collision. The question is: is this approach approved by the ethical
concepts from the paper?
%}

% Create the RRT* path planning algorithm
planner = pathPlannerRRT(costmap,'MinTurningRadius',10);
refPath = plan(planner,startPose,goalPose);

% Refresh the path if it's OK
OK = checkPathValidity(refPath,costmap);
if ~OK
    kimenet = palya;
    disp('Nem tudott létrejönni referenciapálya');
    return
end


% Plot the actual planned path
figure;
plot(planner)

% Create the output vector
palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0]; % The same BUG was here
for i = 1:length(refPath.PathSegments(1,:))
    actual_in_deg = refPath.PathSegments(1,i).GoalPose;
    actual = [actual_in_deg(1), actual_in_deg(2), actual_in_deg(3)*pi/180];
    hossz = palya(i,4) + refPath.PathSegments(1,i).Length;
    palya(i+1,:) = [actual hossz];
end

% The output is the refernce path
kimenet = palya;

% Clear the costmap
clear costmap;
