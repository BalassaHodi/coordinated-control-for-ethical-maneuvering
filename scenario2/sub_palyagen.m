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
    disp('[SUB] Nincs veszély.');
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

% The ahead points of the reference path of the dominant vehicle
% if size(other_palya,1) >= 7 
%     occupiedVal = 1;
%     xyPoint4 = [
%         other_palya(2,1), other_palya(2,2);
%         other_palya(3,1), other_palya(3,2);
%         other_palya(4,1), other_palya(4,2);
%         other_palya(5,1), other_palya(5,2);
%         other_palya(6,1), other_palya(6,2);
%         other_palya(7,1), other_palya(7,2)];
%     setCosts(sub_costmap,xyPoint4,occupiedVal);
% end

% Implement an other kind of technique
% If the refPath of the dominant vehicle is in the danger zone (in the other lane),
% than those positions shall be occupied cells
xyPoint4 = double.empty();
for i = 1:size(other_palya,1)
    if other_palya(i,2) >= 4
        xyPoint4(end+1,:) = [other_palya(i,1), other_palya(i,2)];
    end
end
occupiedVal = 1;
setCosts(sub_costmap,xyPoint4,occupiedVal);


% Plot the costmap for debugging
% figure;
% plot(sub_costmap);



% Set the startPose and the goalPose of the subordinate vehicle
startPose = [input(4), input(5), input(6)*180/pi];  % psi is positive counter clockwise from the negative x-axis
goalPose = sub_goal_pos;


% Check the startpose
sub_OK = checkFree(sub_costmap,[startPose(1), startPose(2), startPose(3)+180]);  % Here the angle starts at the -x axis, so 180 degree shall be added

% If the startPose is bad, than the plan function doesn't work, so we have 
% to use the previous path for safety
if ~sub_OK
    disp('[SUB] A startPose nem megfelelő, így az előző referenciapálya használata...');

    % Work with the previous path
    if t > 2
        % Get the previous palya
        sub_previous_palya = sub_all_palya{t-2};

        % Check the startPose boolean
        startPose_good = false;

        % Check where the startPose is now
        sub_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
        for i = 1:size(sub_previous_palya,1)
            if sub_palya(1,1) > sub_previous_palya(i,1)
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
            for i = 2:(size(sub_previous_palya,1)-startIndex+2)
                sub_palya(i,:) = sub_previous_palya(startIndex+i-2,:);
                sub_OK = checkFree(sub_costmap, [sub_palya(i,1), sub_palya(i,2), 360-(sub_palya(i,3)*180/pi+180)]);
                if sub_OK
                    pathFound = true;
                else
                    pathFound = false;
                    break
                end
            end
        end
    end

    if pathFound
        sub_emergency = false;
        disp('[SUB] Az előző referenciapálya van felhasználva.');
        sub_all_palya{t-1} = sub_palya;
        kimenet = sub_palya;
        % sub_warnings(end+1,:) = {2,'Warning', t-1, 'A startPose nem volt megfelelő, így az előző referenciapálya volt felhasználva.'};
        return
    end
else
    pathFound = false;
end

% If the path couldn't be created, emergency scenario
if ~pathFound && ~sub_OK
    sub_emergency = true;

    % Create the palya vector in case of emergency
    sub_palya = double.empty();
    sub_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
    idx = 2;
    l = 2;
    alpha = sub_palya(1,3);
    while sub_palya(end,1) >= sub_goal_pos(1)
        sub_palya(idx,:) = [sub_palya(idx-1,1)-cos(alpha)*l, sub_palya(idx-1,2)-sin(alpha)*l, sub_palya(1,3), sub_palya(idx-1,4)+l];
        idx = idx + 1;
    end

    kimenet = sub_palya;
    disp('[SUB] Nem tudott létrehönni referenciapálya.');
    % sub_warnings(end+1,:) = {3, 'Error', t-1, 'A startPose nem volt megfelelő, és az előző referenciapályát sem lehetett felhasználni.'};
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
planner = pathPlannerRRT(sub_costmap,'MinTurningRadius',10,'ConnectionDistance',2,'GoalTolerance',[0.5,0.5,5]);

% Since RRT* is a probabilistic algorithm, if the path couldn't be created, try to create it again:
maxAttempts = 10;
pathFound = false;

for attempt = 1:maxAttempts
    refPath = plan(planner,[startPose(1), startPose(2), startPose(3)+180],[goalPose(1), goalPose(2), goalPose(3)+180]);

    sub_OK = checkPathValidity(refPath,sub_costmap);
    if sub_OK
        pathFound = true;
        sub_emergency = false;
        if attempt ~= 1
            % sub_warnings(end+1,:) = {4, 'Info', t-1, sprintf('A referenciapálya létrehozása %d. iterációra történt meg.', attempt)};
        end
        break
    end

    disp(['[SUB] Attempt ', num2str(attempt), ' failed, retrying...']);
end

% Path couldn't be created in this iteration, so try to use the previous path
if ~pathFound
    disp('[SUB] Előző referenciapálya használata...');
    % Work with the previous path
    if t > 2
        % Get the previous palya
        sub_previous_palya = sub_all_palya{t-2};

        % Check the startPose 
        startPose_good = false;

        % Check where the startPose is now
        sub_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
        for i = 1:size(sub_previous_palya,1)
            if sub_palya(1,1) > sub_previous_palya(i,1)
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
            for i = 2:(size(sub_previous_palya,1)-startIndex+2)
                sub_palya(i,:) = sub_previous_palya(startIndex+i-2,:);
                sub_OK = checkFree(sub_costmap,[sub_palya(i,1), sub_palya(i,2), 360-(sub_palya(i,3)*180/pi+180)]);
                if sub_OK
                    pathFound = true;
                else
                    pathFound = false;
                    break
                end
            end
        end
    end

    if pathFound
        sub_emergency = false;
        disp('[SUB] Az előző referenciapálya van felhasználva.');
        sub_all_palya{t-1} = sub_palya;
        kimenet = sub_palya;
        % sub_warnings(end+1,:) = {5, 'Warning', t-1, 'Az időlépésben nem lehetett referenciapályát generálni, így az előző referenciapálya volt felhasználva.'};
        return
    end
end

% If none of the ways was succesful
if ~pathFound
    sub_emergency = true;

    % Create the palya vector in case of emergency
    sub_palya = double.empty();
    sub_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
    idx = 2;
    l = 2;
    alpha = sub_palya(1,3);
    while sub_palya(end,1) >= sub_goal_pos(1)
        sub_palya(idx,:) = [sub_palya(idx-1,1)-cos(alpha)*l, sub_palya(idx-1,2)-sin(alpha)*l, sub_palya(1,3), sub_palya(idx-1,4)+l];
        idx = idx + 1;
    end

    kimenet = sub_palya;
    disp('[SUB] Nem tudott létrejönni referenciapálya.');
    % sub_warnings(end+1,:) = {6, 'Error', t-1, 'Az időlépésben nem lehetett referenciapályát generálni, és az előző referenciapályát sem lehetett felhasználni.'};
    return
end


% If the path was created by RRT* in the actual timestep
% Plot the actual planned path (if there was)
figure;
plot(planner)


% Create the output vector
sub_palya(1,:) = [startPose(1), startPose(2), startPose(3)*pi/180, 0];
for i = 1:length(refPath.PathSegments(1,:))
    actual_in_deg = refPath.PathSegments(1,i).GoalPose;
    actual = [actual_in_deg(1), actual_in_deg(2), actual_in_deg(3)*pi/180-180];
    hossz = sub_palya(i,4) + refPath.PathSegments(1,i).Length;
    sub_palya(i+1,:) = [actual hossz];
end

% Sometimes inside the palya the same elements (x-values) are stored
% So we jave to make sure this doesn't happen
[~,ia,~] = unique(sub_palya(:,1),"stable","last");
removedIndices = setdiff(1:length(sub_palya(:,1)),ia);
for i = 1:length(removedIndices)
    sub_palya(removedIndices(i),:) = [];
    % sub_warnings(end+1,:) = {7, 'Info', t-1, sprintf('Törölni kellett a %d. sort a pályából.',removedIndices(i))};
end

% Store the palya
sub_all_palya{t-1} = sub_palya;

% The output is the reference path
kimenet = sub_palya;


%{
One more improvement:
The occupied cells shall be the actual position of the dominant vehicle and
all the positions that goes to the other lane of the subordinate vehicle.
%}