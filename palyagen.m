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
global all_palya;
global t;

% Clear the palya from the previous iteration
palya = double.empty();

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
Path planning algorithm has been improved. The changes are the following:
1. The connection distance had been set to 2 m instead of the default 5
2. There's a tolerance in the goal position: [2, 0.5, 5], which can be
consodered good.
3. If the refpath could not been created, than it tries to create it 10
more times. If there are 10 failed attempts on creating the path, then the
the path planning is considered to be failed.

One more imporvement can be considered in the path planning algorithm:
Before creating a new path, check that the previous path is good for us,
if yes, than dont create a new path, just follow this one.

Improve path planning algorithm towards safety:
Steps:
1. Get the previous path from all_refPath 
2. Check wether it's still valid
3. If yes, than create the new reference path by:
    3.1. the first element of palya will be the startPose
    3.2. the rest elements are the elements from the PathSegments of the
    previous path

New approach:
Store the palya variable in every step, than work with it, because it's
much more easier.
To check the path validity, just use the checkFree(vehiclePose) function.
%}



% IMPROVE PATH PLANNING
% Create the RRT* path planning algorithm
planner = pathPlannerRRT(costmap,'MinTurningRadius',10,'ConnectionDistance',2,'GoalTolerance',[2, 0.5, 5]);

% Since RRT* is a probabilistic algorithm, if the path couldn't be created, try to create it again:
maxAttempts = 10;
pathFound = false;

for attempt = 1:maxAttempts
    refPath = plan(planner,startPose,goalPose);

    OK = checkPathValidity(refPath,costmap);
    if OK
        pathFound = true;
        break;
    end

    disp(['Attempt ', num2str(attempt), ' failed, retrying...']);
end

% Path couldn't be created in this iteration, so try to use the previous path
if ~pathFound
    disp('Előző referenciapálya használata...');
    % Work with the previous path
    if t > 2
        % Get the previous palya
        previous_palya = all_palya{t-2};
        
        % Check the startPose boolean
        startPose_good = false;

        % Check where the startPose is now
        palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
        for i = 1:size(previous_palya,1)
            if palya(1,1) < previous_palya(i,1)
                startIndex = i;
                startPose_good = true;
                break
            end
        end

        % Check the startPose
        if ~startPose_good
            pathFound = false;
        else
            % Create the new palya array while checking wether the poses are free
            for i = 2:(size(previous_palya,1)-startIndex+2)
                palya(i,:) = previous_palya(startIndex+i-2,:);
                OK = checkFree(costmap,[palya(i,1), palya(i,2), palya(i,3)*pi/180]);
                if OK
                    pathFound = true;
                end
            end
        end
    end

    if pathFound
        disp('Az előző referenciapálya van felhasználva.');
        all_palya{t-1} = palya;
        kimenet = palya;
        clear costmap;
        return
    end
end

% If none of the ways was succesfull:
if ~pathFound
    kimenet = palya;
    disp('Nem tudott létrejönni referenciapálya');
    return
end

% Plot the actual planned path (if there was)
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

% Store the palya
all_palya{t-1} = palya;

% The output is the refernce path
kimenet = palya;

% Clear the costmap
clear costmap;
