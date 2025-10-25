function kimenet=palyagen(input)        %1-3 sajat jmu;   4 masik jmu X
global OK;
global costmap;
mapWidth = 30;
mapLength = 10;
costVal = 0;
cellSize = 0.5;
costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize);%,'InflationRadius',1);

vehicleDims = vehicleDimensions(4.5,1.7);  % 4.5 m long, 1.7 m wide
numCircles = 3;
ccConfig = inflationCollisionChecker(vehicleDims,numCircles);
costmap.CollisionChecker = ccConfig;

occupiedVal = 1;
xyPoint1 = [10,2.5]; 
setCosts(costmap,xyPoint1,occupiedVal);
xyPoint2 = [input(4),7.5; input(4)+2.25,7.5; input(4)+4.5,7.5]; 
setCosts(costmap,xyPoint2,occupiedVal);
%oldalak
occupiedVal = 0.6;
xyPoint3 = [[0:1:mapWidth]' 0*ones(31,1)];
setCosts(costmap,xyPoint3,occupiedVal);
occupiedVal = 0.6;
xyPoint3 = [[0:1:mapWidth]' mapLength*ones(31,1)];
setCosts(costmap,xyPoint3,occupiedVal);
%plot(costmap)

startPose = input(1:3);%[1, 2.5, 0]; % [meters, meters, degrees]
goalPose  = [25, 2.5, 0];

planner = pathPlannerRRT(costmap,'MinTurningRadius',10);
refPath = plan(planner,startPose,goalPose);
OK= checkPathValidity(refPath,costmap);
plot(planner)

%pályaadatok kiszedése
palya(1,:)=[startPose 0];
for i=1:length(refPath.PathSegments(1,:))
    actual=refPath.PathSegments(1,i).GoalPose;
    hossz=palya(i,4)+refPath.PathSegments(1,i).Length;
    palya(i+1,:)=[actual hossz];
end;
kimenet=palya;
clear costmap;
