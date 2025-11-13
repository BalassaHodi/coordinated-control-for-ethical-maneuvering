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
global dom_OK;
global dom_all_palya;
global dom_palya;
global dom_warnings;
global dom_emergency;
global t;
global dom_costmap;
global danger;

% Clear the palya from the previous iteration
dom_palya = double.empty();
danger = false;


% Create the costmap
mapWidth = 30;
mapLength = 10;
costVal = 0;
cellSize = 0.5;
dom_costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize);


% Create the inflated area of the obstacles based on the dimensions of the dominant vehicle
vehicleDims = vehicleDimensions(4.5,1.7);   % 4.5 m long, 1.7 m wide
numCircles = 3;
ccConfig = inflationCollisionChecker(vehicleDims,numCircles);
dom_costmap.CollisionChecker = ccConfig;


% Create the obstacles
% The pedestrian in the middle of the road
occupiedVal = 1;
xyPoint1 = pedestrian;
setCosts(dom_costmap,xyPoint1,occupiedVal);

% The subordinate vehicle in the other lane
xyPoint2 = [input(4)-2*cos(input(6)), input(5)-2*sin(input(6)); input(4), input(5); input(4)+2*cos(input(6)), input(5)+2*sin(input(6))];
setCosts(dom_costmap,xyPoint2,occupiedVal);

% The sides of the roads
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' 0*ones(31,1)];
setCosts(dom_costmap,xyPoint3,occupiedVal);
occupiedVal = 0.6;
xyPoint3 = [(0:1:mapWidth)' mapLength*ones(31,1)];
setCosts(dom_costmap,xyPoint3,occupiedVal);

% Plot the costmap for debugging
% figure;
% plot(dom_costmap);



% Set the startPose and the goalPose of the dominant vehicle
startPose = [input(1), input(2), input(3)*180/pi];  % convert rad to deg
goalPose = dom_goal_pos;


% Check the startpose
dom_OK = checkFree(dom_costmap,startPose);

% If the startPose is bad, than the plan function doesn't work, so we have
% to use the previous path for safety
if ~dom_OK
    disp('A startPose nem megfelelő, így az előző referenciapálya használata...');

    % Work with the previous path
    if t > 2
        % Get the previous palya
        dom_previous_palya = dom_all_palya{t-2};

        % Check the startPose boolean
        startPose_good = false;

        % Check where the startPose is now
        dom_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
        for i = 1:size(dom_previous_palya,1)
            if dom_palya(1,1) < dom_previous_palya(i,1)
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
            for i = 2:(size(dom_previous_palya,1)-startIndex+2)
                dom_palya(i,:) = dom_previous_palya(startIndex+i-2,:);
                dom_OK = checkFree(dom_costmap,[dom_palya(i,1), dom_palya(i,2), 360-dom_palya(i,3)*180/pi]);
                if dom_OK
                    pathFound = true;
                else
                    pathFound = false;
                    break
                end
            end
        end
    end

    if pathFound
        dom_emergency = false;
        disp('Az előző referenciapálya van felhasználva.');
        dom_all_palya{t-1} = dom_palya;
        kimenet = dom_palya;
        % dom_warnings(end+1,:) = {2,'Warning', t-1, 'A startPose nem volt megfelelő, így az előző referenciapálya volt felhasználva.'};

        % If the planned path goes to the other lane, then danger situation is active
        danger = any(dom_palya(:,2) >= 4);
        return
    end
else
    pathFound = false;
end

% If the path couldn't be created, emergency scenario
if ~pathFound && ~dom_OK
    dom_emergency = true;

    % Create the palya vector in case of emergency
    dom_palya = double.empty();
    dom_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
    idx = 2;
    l = 2;
    alpha = dom_palya(1,3);
    while dom_palya(end,1) <= 25
        dom_palya(idx,:) = [dom_palya(idx-1,1)+cos(alpha)*l, dom_palya(idx-1,2)+sin(alpha)*l, dom_palya(1,3), dom_palya(idx-1)+l];
        idx = idx + 1;
    end

    kimenet = dom_palya;
    disp('Nem tudott létrejönni referenciapálya');
    % dom_warnings(end+1,:) = {3, 'Error', t-1, 'A startPose nem volt megfelelő, és az előző referenciapályát sem lehetett felhasználni.'};

    % If the planned path goes to the other lane, then danger situation is active
    danger = any(dom_palya(:,2) >= 4);
    return
end



%{
Path planning algorithm has been improved. The changes are the following:
1. The connection distance had been set to 2 m instead of the default 5
2. There's a tolerance in the goal position: [0.5, 0.5, 5], which can be
considered good.
3. If the refpath could not been created, than it tries to create it 10
more times. If there are 10 failed attempts on creating the path, then a 
safety layer is activated: the previous path is used.

The description of the safety layer (using the previous path):
1. First we check where the startPose is now, to set the next point of the
reference path. If the startPose is not in the reference path, then an error
occurs, and no path is created.
2. We create the new palya (referenca path) exactly from the previous palya,
where the starting point was calculated in the 1st step.
3. We check each reference point wether it's good or not with the checkFree()
function.
4. If the created path from the previous path is good, then that will be
the output of this palyagen.m function.

This safety layer is used in case of two situations:
1. If the refPath couldn't be created in the timestep
2. If the startPose of the vehicle is in a dangerous area, thus the plan()
function cannot be created.

What happens when the reference path couldn't be created at all (even with
the safety layer)?
- Then the concept from the article is activated: the av. is going straight
with maximum deceleration.
%}



% IMPROVE PATH PLANNING
% Create RRT* path planning algorithm
planner = pathPlannerRRT(dom_costmap,'MinTurningRadius',10,'ConnectionDistance',2,'GoalTolerance',[0.5,0.5,5]);

% Since RRT* is a probabilistic algorithm, if the path couldn't be created, try to create it again:
maxAttempts = 10;
pathFound = false;

for attempt = 1:maxAttempts
    refPath = plan(planner,startPose,goalPose);

    dom_OK = checkPathValidity(refPath,dom_costmap);
    if dom_OK
        pathFound = true;
        dom_emergency = false;
        if attempt ~= 1
            % dom_warnings(end+1,:) = {4, 'Info', t-1, sprintf('A referenciapálya létrehozása %d. iterációra történt meg.', attempt)};
        end
        break
    end

    disp(['Attempt ', num2str(attempt), ' failed, retrying...']);
end

% Path couldn't be created in this iteration, so try to use the previous path
if ~pathFound
    disp('Előző referenciapálya használata...');
    % Work with the previous path
    if t > 2
        % Get the previous palya
        dom_previous_palya = dom_all_palya{t-2};

        % Check the startPose os mpw
        startPose_good = false;

        % Check where the startPose is now
        dom_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
        for i = 1:size(dom_previous_palya,1)
            if dom_palya(1,1) < dom_previous_palya(i,1)
                startIndex = i;
                startPose_good = true;
                break
            end
        end
        
        % Check the startPose
        if ~startPose_good
            pathFound = false;
        else
            % Create the new palya array while checking wether poses are free
            for i = 2:(size(dom_previous_palya,1)-startIndex+2)
                dom_palya(i,:) = dom_previous_palya(startIndex+i-2,:);
                dom_OK = checkFree(dom_costmap,[dom_palya(i,1), dom_palya(i,2), 360-dom_palya(i,3)*180/pi]);
                if dom_OK
                    pathFound = true;
                else
                    pathFound = false;
                    break
                end
            end
        end
    end

    if pathFound
        dom_emergency = false;
        disp('Az előző referenciapálya van felhasználva.');
        dom_all_palya{t-1} = dom_palya;
        kimenet = dom_palya;
        % dom_warnings(end+1,:) = {5, 'Warning', t-1, 'Az időlépésben nem lehetett referenciapályát generálni, így az előző referenciapálya volt felhasználva.'};

        % If the planned path goes to the other lane, then danger situation is active
        danger = any(dom_palya(:,2) >= 4);
        return
    end
end

% If none of the ways was succesful:
if ~pathFound
    dom_emergency = true;
    
    % Create the palya vector in case of emergency
    dom_palya = double.empty();
    dom_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
    idx = 2;
    l = 2;
    alpha = dom_palya(1,3);
    while dom_palya(end,1) <= 25
        dom_palya(idx,:) = [dom_palya(idx-1,1)+cos(alpha)*l, dom_palya(idx-1,2)+sin(alpha)*l, dom_palya(1,3), dom_palya(idx-1)+l];
        idx = idx + 1;
    end

    kimenet = dom_palya;
    disp('Nem tudott létrejönni referenciapálya');
    % dom_warnings(end+1,:) = {6, 'Error', t-1, 'Az időlépésben nem lehetett referenciapályát generálni, és az előző referenciapályát sem lehetett felhasználni.'};

    % If the planned path goes to the other lane, then danger situation is active
    danger = any(dom_palya(:,2) >= 4);
    return
end


% If the path was created by RRT in the actual timestep
% Plot the actual planned path (if there was)
figure;
plot(planner)


% Create the output vector
dom_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
for i = 1:length(refPath.PathSegments(1,:))
    actual_in_deg = refPath.PathSegments(1,i).GoalPose;
    actual = [actual_in_deg(1), actual_in_deg(2), actual_in_deg(3)*pi/180];
    hossz = dom_palya(i,4) + refPath.PathSegments(1,i).Length;
    dom_palya(i+1,:) = [actual hossz];
end

% Sometimes inside the palya the same elements (x-values) are stored
% So we have to make sure this doesn't happen
[uniquePalyaX,ia,ic] = unique(dom_palya(:,1),"stable","last");
removedIndices = setdiff(1:length(dom_palya(:,1)),ia);
for i = 1:length(removedIndices)
    dom_palya(removedIndices(i),:) = [];
    % dom_warnings(end+1,:) = {7, 'Info', t-1, sprintf('Törölni kellett a %d. sort a pályából.',removedIndices(i))};
end

% Store the palya 
dom_all_palya{t-1} = dom_palya;

% The output is the reference path
kimenet = dom_palya;

% If the planned path goes to the other lane, then danger situation is active
danger = any(dom_palya(:,2) >= 4);



%{
One more thing is important to take into consideration:
If the ref_path does not go to the opposite lane, then the av. on the opposite
lane should just go forward without using any path planning algorithm.

In order to do this I have to make a variable that watches the output wether
it generates path to the other lane or not
%}