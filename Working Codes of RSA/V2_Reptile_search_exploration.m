clc;
clear;
close all;
nRbt =3;
nOrient = 2;
nIter = 50;
bound = 20;
d = 1; %step


SearchAgents_no=30;

% 
a = 2*randn;
adamp = a/nIter;
dim = 1;
varSize = [1 dim];

ray_lngth = 1.5; % max_range value in the paper


Vrobot = 1;

anglesClsion = [pi/2,pi/4,0,7*pi/4,3*pi/2,5*pi/4,pi,3*pi/4];
thresholdClsionDttn = 0.9; %threshold of collition detection
maxrangeClsion = 2; %maxrange to detect collition

%%
mapsize = 20;
inflatesize = 0.5;
nItern = 2300;
dim = 2;
varSize = [1 dim];

map = robotics.BinaryOccupancyGrid(mapsize+10,mapsize,2);
%%
% obstacle's coordinate

A(1:60) = 0:inflatesize:29.8;
B(1:60) = 0.4;
C = [A;B];
E = [B(1:40);A(1:40)];
C = transpose(C);
C(11:20,:) = 0;
E = transpose(E);
F(1:60) = 19.7;
G = [A;F];
G = transpose(G);
Ff(1:40) = 29.7;
H = [Ff;A(1:40)];
H = transpose(H);
I(1:20) = 20:inflatesize:29.5;
J(1:20) = 12;
K = [I;J];
K = transpose(K);
L(1:10) = 5;
Ii(1:10) = 20:inflatesize:24.5;
M = [Ii;L];
M = transpose(M);

N(1:12) = 9;
O(1:12) = 0:inflatesize:5.5;
P = [N;O];
P = transpose(P);
S(1:10) = 15:inflatesize:19.5;
Nn = N(1:10);
T = [Nn;S];
T = transpose(T);


% C is below horizontal wall
setOccupancy(map, C, 1)
% E is left vertical wall
setOccupancy(map, E, 1)
% G is upper horizontal wall
setOccupancy(map, G, 1)
% H is right vertical wall
setOccupancy(map, H, 1)
% obstacles inside environment
setOccupancy(map, P, 1)
setOccupancy(map, T, 1)
inflate(map, 0.3)
% thin obstacles
setOccupancy(map, K, 1)
setOccupancy(map, M, 1)
inflate(map, 0.3)


% input parameter of ExampleHelperRobotSimulator should be
% BinaryOccupancyGrid object
robot = ExampleHelperRobotSimulator(map);

grid on;



setRobotPose(robot,[3 4 0]);
enableROSInterface(robot,true);
robot.LaserSensor.NumReadings = 50;
% enableLaser(sim,true);
% showTrajectory (sim,true);
%%

% we need to get simulator laser data. that's why we should subscribe the
% scan topic
scanSub = rossubscriber('scan');

% publisher for sending velocity command
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
path = [5 5];
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.8;
controller.MaxAngularVelocity = 0.8;
% rostf allows to find robot pose at the time when laser is observing
tftree = rostf;
pause(1)

% map = robotics.OccupancyGrid(mapsize+10,mapsize,10);
% figureHandle = figure('Name', 'Map');
% axesHandle = axes('Parent', figureHandle);
% mapHandle = show(map, 'Parent', axesHandle);
% title(axesHandle, 'OccupancyGrid: Update 0');

controlRate = robotics.Rate(10);
% value of laser sensor
group1 = robot.LaserSensor.AngleSweep(1:1:5);
% laser index
group1_inx = 1:5;

% number is negative, sharp right side
group2 = robot.LaserSensor.AngleSweep(6:1:20);
group2_inx = 6:20;

% number is negative, right side
group3 = robot.LaserSensor.AngleSweep(21:1:30);
group3_inx = 21:30;

% number is positive, left side
group4 = robot.LaserSensor.AngleSweep(31:1:45);
group4_inx = 31:45;

% number is positive, sharp left side
group5 = robot.LaserSensor.AngleSweep(46:1:50);
group5_inx = 46:50;

pose = getTransform(tftree, 'map', 'robot_base', scanMsg.Header.Stamp, 'Timeout', 2);
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
                   pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    x = robotPose(1,1);
    y = robotPose(1,2);
    
    scan = lidarScan(scanMsg);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = robot.LaserSensor.MaxRange;
    modScan = lidarScan(ranges, scan.Angles);
    rangesNotNan = scan.Ranges;
    rangesNotNan(isnan(rangesNotNan)) = 5;

%%


% map = robotics.OccupancyGrid(bound, bound, 15); 
% show(map);
% hold on;
% %%
% cntx = bound/2;
% cnty = bound/2;
% s = cnty/0.1;
% A(1,s) = cntx-1;
% A(1:s) = cntx;
% B = cnty:0.1:bound-0.1;
% C = [A; B];
% C = transpose(C);
% A = A/2.5;
% D = [B; A];
% D = transpose(D);
% % q = (bound/0.1) - 10;
% q = (bound/0.1);
% E(1,q) = bound-1;
% E(1:q) = 0.1;
% I(1,q) = bound-1;
% I(1:q) = bound-0.1;
% % F = 1:0.1:bound-0.1;
% F = 0:0.1:bound-0.1;
% H = [E;F];
% H = transpose(H);
% G = [I;F];
% G = transpose(G);
% J = [F;I];
% J = transpose(J);
% W = [F;E];
% W = transpose(W);
% 
% d_ = 0;
% % S = 15:0.1:20;
% % S_(1:51) = 5;
% % S__ = [S;S_];
% % obs_1 = transpose(S__);
% 
% Y = 0:0.1:2;
% Y_(1:21) = 14;
% Y__ = [Y;Y_];
% obs_2 = transpose(Y__);
% 
% O(1:21) = 15;
% O_ = 18:0.1:20;
% O__ = [O;O_];
% obs_7 = transpose(O__);
% 
% R = 7:0.1:9;
% R_(1:21) = 1;
% R__ = [R;R_];
% obs_8 = transpose(R__);
% 
%  setOccupancy(map, W, 1) % lower boundary
% %setOccupancy(map, obs_8, 1)
% 
% % upper vertical
% %setOccupancy(map, C, 1)
%  
% % H left vertical
% setOccupancy(map, H, 1);
% % G right vertical
% setOccupancy(map, G, 1)  % righthand boundary
% % J upper horizontal
% setOccupancy(map, J, 1)  % top boundary
% %  setOccupancy(map, W, 1)
% 
% %setOccupancy(map, obs_1, 1)
% inflate(map, 1); % Inflate by a certain radius to account for the robot's size
% 
% show(map)
% hold on;
% 
% 
% 
% 
% % length
% CS = 2;
% 
% % Positions
% angles = (0:5) * pi/3;
% xCoords = 10 + 5 * cos(angles);
% yCoords = 10 + 5 * sin(angles);
% 
% % Define coordinates and drw them
% for i = 1:6
%     
%     x = [xCoords(i)-CS/2, xCoords(i)+CS/2, xCoords(i)+CS/2, xCoords(i)-CS/2, xCoords(i)-CS/2]; 
%     y = [yCoords(i)-CS/2, yCoords(i)-CS/2, yCoords(i)+CS/2, yCoords(i)+CS/2, yCoords(i)-CS/2];
%     z = [0, 0, 0, 0, 0; CS, CS, CS, CS, CS];
%     
%    
%     fill3(x, y, z(1,:), 'k')
%     fill3(x, y, z(2,:), 'k')
%     fill3([x(1),x(1),x(4),x(4)], [y(1),y(1),y(4),y(4)], [0, CS, CS, 0], 'k')
%     fill3([x(2),x(2),x(3),x(3)], [y(2),y(2),y(3),y(3)], [0, CS, CS, 0], 'k')
%     fill3([x(1),x(1),x(2),x(2)], [y(1),y(1),y(2),y(2)], [0, CS, CS, 0], 'k')
%     fill3([x(3),x(3),x(4),x(4)], [y(3),y(3),y(4),y(4)], [0, CS, CS, 0], 'k')
% end
% 
% % Define coordinates and draw them
% bh = 4;
% bw = 2;
% for i = 1:6
%     
%     x = [xCoords(i)-bw/2, xCoords(i)+bw/2, xCoords(i)+bw/2, xCoords(i)-bw/2, xCoords(i)-bw/2]; 
%     y = [yCoords(i)-bw/2, yCoords(i)-bw/2, yCoords(i)+bw/2, yCoords(i)+bw/2, yCoords(i)-bw/2];
%     z = [CS, CS, CS, CS, CS; CS + bh, CS + bh, CS + bh, CS + bh, CS + bh];
% 
%    
%     fill3(x, y, z(1,:), 'k')
%     fill3(x, y, z(2,:), 'k')
%     fill3([x(1),x(1),x(4),x(4)], [y(1),y(1),y(4),y(4)], [CS, CS + bh, CS + bh, CS], 'k')
%     fill3([x(2),x(2),x(3),x(3)], [y(2),y(2),y(3),y(3)], [CS, CS + bh, CS + bh, CS], 'k')
%     fill3([x(1),x(1),x(2),x(2)], [y(1),y(1),y(2),y(2)], [CS, CS + bh, CS + bh, CS], 'k')
%     fill3([x(3),x(3),x(4),x(4)], [y(3),y(3),y(4),y(4)], [CS, CS + bh, CS + bh, CS], 'k')
% end
% 
% % Define coordinates
% cx = 10;
% ccy = 2;
% cc = 0;
% w = 8;
% m = 0.5;
% n = 2;
% 
% 
% x = [cx-w/2, cx+w/2, cx+w/2, cx-w/2, cx-w/2]; 
% y = [ccy-m/2, ccy-m/2, ccy+m/2, ccy+m/2, ccy-m/2];
% z = [cc, cc, cc, cc, cc; cc + n, cc + n, cc + n, cc + n, cc + n];
% 
% 
% fill3(x, y, z(1,:), 'k')
% fill3(x, y, z(2,:), 'k')
% fill3([x(1),x(1),x(4),x(4)], [y(1),y(1),y(4),y(4)], [cc, cc + n, cc + n, cc], 'k')
% fill3([x(2),x(2),x(3),x(3)], [y(2),y(2),y(3),y(3)], [cc, cc + n, cc + n, cc], 'k')
% fill3([x(1),x(1),x(2),x(2)], [y(1),y(1),y(2),y(2)], [cc, cc + n, cc + n, cc], 'k')
% fill3([x(3),x(3),x(4),x(4)], [y(3),y(3),y(4),y(4)], [cc, cc + n, cc + n, cc], 'k')
% 
% view(3);



%% tunnel obstacle
% right horizontal
% setOccupancy(map, D, 1)
% map = binaryOccupancyMap(100,80,1);
% occ = zeros(80,100);
% occ(1,:) = 1;
% occ(end,:) = 1;
% occ([1:30,51:80],1) = 1;
% occ([1:30,51:80],end) = 1;
% occ(40,20:80) = 1;
% occ(28:52,[20:21 32:33 44:45 56:57 68:69 80:81]) = 1;
% occ(1:12,[20:21 32:33 44:45 56:57 68:69 80:81]) = 1;
% occ(end-12:end,[20:21 32:33 44:45 56:57 68:69 80:81]) = 1;
% 
% helperAddObstacle(map,5,5,[10,30]);
% helperAddObstacle(map,5,5,[20,17]);
% helperAddObstacle(map,5,5,[40,17]);
% 
%  setOccupancy(map,occ)

%% structure 

ranges = ray_lngth*ones(100,1); % 1.5 cells of range sensor beam
angles = linspace(3.14, -3.13, 100);
% angles = linspace(2*pi, 100);
maxrange = 5;
gridmap_arr_empty.Itertn.map = [];
gridmap_arr = repmat(gridmap_arr_empty,nRbt,1);


robot_empty.Itertn.Cost = [];
robot_empty.Itertn.Pose = [];
robot_empty.Itertn.V1.V1xy = [];
robot_empty.Itertn.V2.V2xy = [];
robot_empty.Itertn.V3.V3xy = [];
robot_empty.Itertn.V4.V4xy = [];
robot_empty.Itertn.V5.V5xy = [];
robot_empty.Itertn.V6.V6xy = [];
robot_empty.Itertn.V7.V7xy = [];
robot_empty.Itertn.V8.V8xy = [];
robot_empty.Itertn.V9.V9xy = [];
robot_empty.Itertn.V1.V1cost = [];
robot_empty.Itertn.V2.V2cost = [];
robot_empty.Itertn.V3.V3cost = [];
robot_empty.Itertn.V4.V4cost = [];
robot_empty.Itertn.V5.V5cost = [];
robot_empty.Itertn.V6.V6cost = [];
robot_empty.Itertn.V7.V7cost = [];
robot_empty.Itertn.V8.V8cost = [];
robot_empty.Itertn.V8.V9cost = [];
robot_empty.Itertn.MinCost.Value = [];
robot_empty.Itertn.MinCost.Index = [];
robot_empty.Itertn.MinCost.Vxy = [];
robot_empty.Itertn.V1.V1Utility = [];
robot_empty.Itertn.V2.V2Utility = [];
robot_empty.Itertn.V3.V3Utility = [];
robot_empty.Itertn.V4.V4Utility = [];
robot_empty.Itertn.V5.V5Utility = [];
robot_empty.Itertn.V6.V6Utility = [];
robot_empty.Itertn.V7.V7Utility = [];
robot_empty.Itertn.V8.V8Utility = [];
robot_empty.Itertn.V9.V9Utility = [];
robot_empty.Itertn.V1.V1Vbest = [];
robot_empty.Itertn.V2.V2Vbest = [];
robot_empty.Itertn.V3.V3Vbest = [];
robot_empty.Itertn.V4.V4Vbest = [];
robot_empty.Itertn.V5.V5Vbest = [];
robot_empty.Itertn.V6.V6Vbest = [];
robot_empty.Itertn.V7.V7Vbest = [];
robot_empty.Itertn.V8.V8Vbest = [];
robot_empty.Itertn.V9.V9Vbest = [];
robot_empty.Itertn.V1.V1Index = [];
robot_empty.Itertn.V2.V2Index = [];
robot_empty.Itertn.V3.V3Index = [];
robot_empty.Itertn.V4.V4Index = [];
robot_empty.Itertn.V5.V5Index = [];
robot_empty.Itertn.V6.V6Index = [];
robot_empty.Itertn.V7.V7Index = [];
robot_empty.Itertn.V8.V8Index = [];
robot_empty.Itertn.V9.V9Index = [];
robot_empty.Itertn.MaxUtility.Value = [];
robot_empty.Itertn.MaxUtility.Index = [];
robot_empty.Itertn.MaxUtility.Uxy = [];
obstacle = [];
V_empty.Itertn.V = [];
Vblock_empty.Itertn.V = [];
robots = repmat(robot_empty, nRbt, 1);
V = repmat(V_empty, nRbt, 1);
Vblock = repmat(Vblock_empty, nRbt, 1);

Utility = ones(bound);

reptile_empty.Itertn.crocodile.Position = [];
reptile_empty.Itertn.crocodile.Cost = [];
reptile_empty.Itertn.crocodile.Index = [];

 reptile_empty.Itertn.crocodile2.Position = [];
 reptile_empty.Itertn.crocodile2.Cost = [];
 reptile_empty.Itertn.crocodile2.Index = [];
 
 reptile_empty.Itertn.crocodile3.Position = [];
 reptile_empty.Itertn.crocodile3.Cost = [];
 reptile_empty.Itertn.crocodile3.Index = [];

reptile_empty.Itertn.crocodile4.Position = [];
 reptile_empty.Itertn.crocodile4.Cost = [];
 reptile_empty.Itertn.crocodile4.Index = [];


reptileX_empty.Itertn.Pairs.X = [];
 reptileX_empty.Itertn.Pairs.Combination = [];
 reptileX_empty.Itertn.Pairs.Cost = [];
reptileX = repmat(reptileX_empty,nRbt,1);
reptile = repmat(reptile_empty,nRbt,1);
% Utility = rand(20,bound);
% %% obstacle array
% for i = 1:bound
%     for j = 1:bound
%         obstacle(
%     end
% end
counter = 1;

for i = 1:bound
    for j = 1:bound
    
    
        occ = getOccupancy(map, [i, j]);
        if occ > 0.8000
            obstacle(counter, :) = [i, j];
            counter = counter + 1;
        end
    end
    
end
% obstacle
% for row = 2:bound-2
%     for column = 2:bound-2
%         Utility_total =  Utility_total + Utility(row, column);
%     end
% end
% Utility_total = Utility_total - 30;

%% initialization in Itertn 1
for i=1
   % for j=1:nRbt
%         if j==1
        %    robots(j).Itertn(i).Pose = [3,4];
  
%         end
%         if j==2
%             robots(j).Itertn(i).Pose = [5,4];
% %             robots(j).Itertn(i).Pose = [7,4,0];
%         end
%         if j==3
%             robots(j).Itertn(i).Pose = [7,4];
% %             robots(j).Itertn(i).Pose = [8,4,0];
%         end
%         if j==4
%             robots(j).Itertn(i).Pose = [7,15];
% %             robots(j).Itertn(i).Pose = [8,4,0];
%         end


        robots(j).Itertn(i).Cost = [0, 0, 0, 0, 0, 0, 0, 0, 1];
        robots(j).Itertn(i).MaxUtility.Uxy = [0,0];
        robots(j).Itertn(i).MinCost.Vxy = [0,0];
        robots(j).Itertn(i).V1.V1Index = 1;
        robots(j).Itertn(i).V2.V2Index = 2;
        robots(j).Itertn(i).V3.V3Index = 3;
        robots(j).Itertn(i).V4.V4Index = 4;
        robots(j).Itertn(i).V5.V5Index = 5;
        robots(j).Itertn(i).V6.V6Index = 6;
        robots(j).Itertn(i).V7.V7Index = 7;
        robots(j).Itertn(i).V8.V8Index = 8;
        robots(j).Itertn(i).V9.V9Index = 9;
        
        robots(j).Itertn(i).V1.V1Utility = 1;
        robots(j).Itertn(i).V2.V2Utility = 1;
        robots(j).Itertn(i).V3.V3Utility = 1;
        robots(j).Itertn(i).V4.V4Utility = 1;
        robots(j).Itertn(i).V5.V5Utility = 1;
        robots(j).Itertn(i).V6.V6Utility = 1;
        robots(j).Itertn(i).V7.V7Utility = 1;
        robots(j).Itertn(i).V8.V8Utility = 1;
        robots(j).Itertn(i).V9.V9Utility = 1;
        
        robots(j).Itertn.V1.V1Vbest = 0;
        robots(j).Itertn.V2.V2Vbest = 0;
        robots(j).Itertn.V3.V3Vbest = 0;
        robots(j).Itertn.V4.V4Vbest = 0;
        robots(j).Itertn.V5.V5Vbest = 0;
        robots(j).Itertn.V6.V6Vbest = 0;
        robots(j).Itertn.V7.V7Vbest = 0;
        robots(j).Itertn.V8.V8Vbest = 0;
        robots(j).Itertn.V9.V9Vbest = 0;
        
        
        
        for row = 1:21
            for col = 1:21
                gridmap_arr(j).Itertn(i).map(row,col) = getOccupancy(map,[row-1,col-1]);

            end
        end

        

     %   x = robots(j).Itertn(i).Pose(1,1);
     %   y = robots(j).Itertn(i).Pose(1,2);
%          thi = robots(j).Itertn(i).Pose(1,3);        
      %  insertRay(map,[x,y,0],ranges,angles,maxrange); 
        hold on
        
        color = ['k','m','c','y'];
        if j==1
%            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),...
     %           'color',color(1),'marker','x');
%             hold on
%         elseif j==2
%             plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),...
%                 'color',color(2),'marker','p');
%             hold on
%         elseif j==3
%             plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),...
%                 'color',color(3),'marker','*');
%             hold on
%         elseif j==4
%             plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),...
%                 'color',color(4),'marker','+');
            hold on
        end


        show(map)
      %  view(3)
        grid on
        hold on
   % end
end

       

k_ = 1;

% 
%% Main source
% 
%Cost
hold on
% try 

    for i=1:nIter
        val = 1;
        for j=1:nRbt       
    %         if i ~= 1
        %        x = robots(j).Itertn(i).Pose(1,1);
        %        y = robots(j).Itertn(i).Pose(1,2);
                hold on
     %           insertRay(map,[x,y,0],ranges,angles,maxrange);
    %         end


            [V1,V2,V3,V4,V5,V6,V7,V8,V9] = gridsOrientation(x,y);
            V(j).Itertn(i).V = [V1;V2;V3;V4;V5;V6;V7;V8;V9];
            robots(j).Itertn(i).V1.V1Index = 1;
            robots(j).Itertn(i).V2.V2Index = 2;
            robots(j).Itertn(i).V3.V3Index = 3;
            robots(j).Itertn(i).V4.V4Index = 4;
            robots(j).Itertn(i).V5.V5Index = 5;
            robots(j).Itertn(i).V6.V6Index = 6;
            robots(j).Itertn(i).V7.V7Index = 7;
            robots(j).Itertn(i).V8.V8Index = 8;
            robots(j).Itertn(i).V9.V9Index = 9;


            robots(j).Itertn(i).V1.V1xy = V1;
            robots(j).Itertn(i).V2.V2xy = V2;
            robots(j).Itertn(i).V3.V3xy = V3;
            robots(j).Itertn(i).V4.V4xy = V4;
            robots(j).Itertn(i).V5.V5xy = V5;
            robots(j).Itertn(i).V6.V6xy = V6;
            robots(j).Itertn(i).V7.V7xy = V7;
            robots(j).Itertn(i).V8.V8xy = V8;
            robots(j).Itertn(i).V9.V9xy = V9; 


            % cost
            Vbst =  0;
            if i == 1
                robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i).Cost(1,1) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i).Cost(1,2) + (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i).Cost(1,3) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i).Cost(1,4) + (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i).Cost(1,5) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i).Cost(1,6) + (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i).Cost(1,7) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i).Cost(1,8) + (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i).Cost(1,9) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));

                robots(j).Itertn(i).Cost = [robots(j).Itertn(i).V1.V1cost, robots(j).Itertn(i).V2.V2cost, robots(j).Itertn(i).V3.V3cost, robots(j).Itertn(i).V4.V4cost,...
                    robots(j).Itertn(i).V5.V5cost, robots(j).Itertn(i).V6.V6cost, robots(j).Itertn(i).V7.V7cost, robots(j).Itertn(i).V8.V8cost, robots(j).Itertn(i).V9.V9cost];

                [robots(j).Itertn(i).MinCost.Value, robots(j).Itertn(i).MinCost.Index] = min(robots(j).Itertn(i).Cost(robots(j).Itertn(i).Cost>0));

                U1 = Utility(robots(j).Itertn(i).V1.V1xy(1,1), robots(j).Itertn(i).V1.V1xy(1,2)) - robots(j).Itertn(i).Cost(1,1);
                U2 = Utility(robots(j).Itertn(i).V2.V2xy(1,1), robots(j).Itertn(i).V2.V2xy(1,2)) - robots(j).Itertn(i).Cost(1,2);
                U3 = Utility(robots(j).Itertn(i).V3.V3xy(1,1), robots(j).Itertn(i).V3.V3xy(1,2)) - robots(j).Itertn(i).Cost(1,3);
                U4 = Utility(robots(j).Itertn(i).V4.V4xy(1,1), robots(j).Itertn(i).V4.V4xy(1,2)) - robots(j).Itertn(i).Cost(1,4);
                U5 = Utility(robots(j).Itertn(i).V5.V5xy(1,1), robots(j).Itertn(i).V5.V5xy(1,2)) - robots(j).Itertn(i).Cost(1,5);
                U6 = Utility(robots(j).Itertn(i).V6.V6xy(1,1), robots(j).Itertn(i).V6.V6xy(1,2)) - robots(j).Itertn(i).Cost(1,6);
                U7 = Utility(robots(j).Itertn(i).V7.V7xy(1,1), robots(j).Itertn(i).V7.V7xy(1,2)) - robots(j).Itertn(i).Cost(1,7);
                U8 = Utility(robots(j).Itertn(i).V8.V8xy(1,1), robots(j).Itertn(i).V8.V8xy(1,2)) - robots(j).Itertn(i).Cost(1,8);

                allUtility = [U1,U2,U3,U4,U5,U6,U7,U8];
                [robots(j).Itertn(i).MaxUtility.Value, robots(j).Itertn(i).MaxUtility.Index] = max(allUtility);

                Vblock(j).Itertn(i).V = [0,0];
            end
            if i>1
                val = 1;
                if j==2 || j==3
                    k_ = 1;
                    Vblock(j).Itertn(i).V = [0,0];
                    for i_=1:9  
    %                     Vblock(j).Itertn(i).V = [0,0];
                        for j_=1:9

                            if V(j).Itertn(i).V(i_,1)==V(j-1).Itertn(i).V(j_,1) && V(j).Itertn(i).V(i_,2)==V(j-1).Itertn(i).V(j_,2)
    %                             disp([V(j).Itertn(i).V(i_,1),V(j).Itertn(i).V(i_,2)]);
                                Vblock(j).Itertn(i).V(k_,:) = [V(j).Itertn(i).V(i_,1), V(j).Itertn(i).V(i_,2)];
                                k_ = k_ + 1;

                            end
                            if j == 3
                                   if V(3).Itertn(i).V(i_,1)==V(1).Itertn(i).V(j_,1) && V(3).Itertn(i).V(i_,2)==V(1).Itertn(i).V(j_,2)
    %                                    disp([V(j).Itertn(i).V(i_,1),V(j).Itertn(i).V(i_,2)]);
                                       Vblock(j).Itertn(i).V(k_,:) = [V(j).Itertn(i).V(i_,1), V(j).Itertn(i).V(i_,2)];
                                       k_ = k_ + 1;
                                   end
                            end




                        end

                    end
                else
                    Vblock(j).Itertn(i).V = [0,0];
                end




                if robots(j).Itertn(i-1).MaxUtility.Index == 1
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m  

                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = (pdist([x,y;x,y+1],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V1.V1cost = (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));

                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end

                   for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                   end

                   for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                   end

                   for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x,y],'euclidean'))*0.7;
                            break;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end

                end
                if robots(j).Itertn(i-1).MaxUtility.Index == 2
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end
                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end
                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end
                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end
                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end
                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = Vbst + (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = Vbst + (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                    end
                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end
                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 3
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end

                    for cnt = 1:m                
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m                
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m                
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m                
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m                
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end
                    for cnt = 1:m                
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end
                    for cnt = 1:m                
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                    end
                    for cnt = 1:m                
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end
                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 4
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end
                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end

                   for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                   end

                   for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end

                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 5
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end

                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                    end

                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end
                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 6
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end

                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                    end

                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end

                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 7
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end

                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                    end

                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x,y],'euclidean'))*0.9;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end

                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 8
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V1.V1cost = (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V2.V2cost = (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
                        end
                    end

                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
                        end
                    end

                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
                        end
                    end
                end

                if robots(j).Itertn(i-1).MaxUtility.Index == 9
                    [m,n] = size(Vblock(j).Itertn(i).V);
                    for cnt = 1:m               
                        if V1(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V1(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x,y+1],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x,y+1],'euclidean'))*(getOccupancy(map,V1));
    %                         robots(j).Itertn(i).V1.V1cost = robots(j).Itertn(i-1).Cost(1,1) + (pdist([x,y;x,y+1],'euclidean'))*0;
                            robots(j).Itertn(i).V1.V1cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V2(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V2(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x+1,y+1],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x+1,y+1],'euclidean'))*(getOccupancy(map,V2));
    %                           robots(j).Itertn(i).V2.V2cost = robots(j).Itertn(i-1).Cost(1,2) + (pdist([x,y;x+1,y+1],'euclidean'))*0;
                              robots(j).Itertn(i).V2.V2cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V3(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V3(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V3.V3cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
                              robots(j).Itertn(i).V4.V4cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V5.V5cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V6.V6cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V7.V7cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
                            robots(j).Itertn(i).V8.V8cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
                              robots(j).Itertn(i).V9.V9cost = 0;
                        end
                    end

                end



                robots(j).Itertn(i).Cost = [robots(j).Itertn(i).V1.V1cost, robots(j).Itertn(i).V2.V2cost, robots(j).Itertn(i).V3.V3cost, robots(j).Itertn(i).V4.V4cost,...
                robots(j).Itertn(i).V5.V5cost, robots(j).Itertn(i).V6.V6cost, robots(j).Itertn(i).V7.V7cost, robots(j).Itertn(i).V8.V8cost, ...
                robots(j).Itertn(i).V9.V9cost];


                [robots(j).Itertn(i).MinCost.Value, robots(j).Itertn(i).MinCost.Index] = min(robots(j).Itertn(i).Cost(robots(j).Itertn(i).Cost>0));





            end

        end

        % For reptile
            crocodile1 = inf;
           crocodile2 = inf;
            crocodile3 = inf;
crocodile4 = inf;
        %%  reduce Utilities
        j = 1;
        for j=1:nRbt

            Utility(robots(j).Itertn(i).V1.V1xy(1,1), robots(j).Itertn(i).V1.V1xy(1,2)) = Utility(robots(j).Itertn(i).V1.V1xy(1,1), robots(j).Itertn(i).V1.V1xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V1.V1xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V2.V2xy(1,1), robots(j).Itertn(i).V2.V2xy(1,2)) = Utility(robots(j).Itertn(i).V2.V2xy(1,1), robots(j).Itertn(i).V2.V2xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V2.V2xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V3.V3xy(1,1), robots(j).Itertn(i).V3.V3xy(1,2)) = Utility(robots(j).Itertn(i).V3.V3xy(1,1), robots(j).Itertn(i).V3.V3xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V3.V3xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V4.V4xy(1,1), robots(j).Itertn(i).V4.V4xy(1,2)) = Utility(robots(j).Itertn(i).V4.V4xy(1,1), robots(j).Itertn(i).V4.V4xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V4.V4xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V5.V5xy(1,1), robots(j).Itertn(i).V5.V5xy(1,2)) = Utility(robots(j).Itertn(i).V5.V5xy(1,1), robots(j).Itertn(i).V5.V5xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V5.V5xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V6.V6xy(1,1), robots(j).Itertn(i).V6.V6xy(1,2)) = Utility(robots(j).Itertn(i).V6.V6xy(1,1), robots(j).Itertn(i).V6.V6xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V6.V6xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V7.V7xy(1,1), robots(j).Itertn(i).V7.V7xy(1,2)) = Utility(robots(j).Itertn(i).V7.V7xy(1,1), robots(j).Itertn(i).V7.V7xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V7.V7xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V8.V8xy(1,1), robots(j).Itertn(i).V8.V8xy(1,2)) = Utility(robots(j).Itertn(i).V8.V8xy(1,1), robots(j).Itertn(i).V8.V8xy(1,2)) - abs(getOccupancy(map,robots(j).Itertn(i).V8.V8xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V9.V9xy(1,1), robots(j).Itertn(i).V9.V9xy(1,2)) = 0;

          % reducing the utility of 8 neighbor cells in order to find max
          % utilities; 
             U1 = Utility(robots(j).Itertn(i).V1.V1xy(1,1), robots(j).Itertn(i).V1.V1xy(1,2)) - robots(j).Itertn(i).Cost(1,1);
             U2 = Utility(robots(j).Itertn(i).V2.V2xy(1,1), robots(j).Itertn(i).V2.V2xy(1,2)) - robots(j).Itertn(i).Cost(1,2);
             U3 = Utility(robots(j).Itertn(i).V3.V3xy(1,1), robots(j).Itertn(i).V3.V3xy(1,2)) - robots(j).Itertn(i).Cost(1,3);
             U4 = Utility(robots(j).Itertn(i).V4.V4xy(1,1), robots(j).Itertn(i).V4.V4xy(1,2)) - robots(j).Itertn(i).Cost(1,4);
             U5 = Utility(robots(j).Itertn(i).V5.V5xy(1,1), robots(j).Itertn(i).V5.V5xy(1,2)) - robots(j).Itertn(i).Cost(1,5);
             U6 = Utility(robots(j).Itertn(i).V6.V6xy(1,1), robots(j).Itertn(i).V6.V6xy(1,2)) - robots(j).Itertn(i).Cost(1,6);
             U7 = Utility(robots(j).Itertn(i).V7.V7xy(1,1), robots(j).Itertn(i).V7.V7xy(1,2)) - robots(j).Itertn(i).Cost(1,7);
             U8 = Utility(robots(j).Itertn(i).V8.V8xy(1,1), robots(j).Itertn(i).V8.V8xy(1,2)) - robots(j).Itertn(i).Cost(1,8);
    %             U9 = Utility(robots(j).Itertn(i).V9.V9xy(1,1), robots(j).Itertn(i).V9.V9xy(1,2)) - robots(j).Itertn(i).Cost(1,9);


             allUtility = [U1,U2,U3,U4,U5,U6,U7,U8];
             [robots(j).Itertn(i).MaxUtility.Value, robots(j).Itertn(i).MaxUtility.Index] = max(allUtility);

            t_ = robots(j).Itertn(i).MaxUtility.Index;
            V_ = eval(['robots(j).Itertn(i).V' num2str(t_)]);
            V__ = eval(['V_.V' num2str(t_) 'xy']);
            robots(j).Itertn(i).MaxUtility.Uxy = V__;


            x_ = robots(j).Itertn(i).Pose(1,1);
            y_ = robots(j).Itertn(i).Pose(1,2);
            [V1,V2,V3,V4,V5,V6,V7,V8,V9] = gridsOrientation(x_,y_);
    %         Utl = [Utility(V1(1,1),V1(1,2)),Utility(V2(1,1),V2(1,2)),Utility(V3(1,1),V3(1,2)),Utility(V4(1,1),V4(1,2)),...
    %             Utility(V5(1,1),V5(1,2)),Utility(V6(1,1),V6(1,2)),Utility(V7(1,1),V7(1,2)),Utility(V8(1,1),V8(1,2))];

            [val,idx] = sort(allUtility,'descend');  %sorting utilities (three max values)
            % since we have 3 robots each robot needs to supplemented with
            % somevalue therefeore we keep first three obtained value and
            % feed them to first three crocodiles
            crocodile = val(1);
            crocodile_index = idx(1);
            crocodile2 = val(2);
            crocodile2_index = idx(2);
            crocodile3 = val(3);
            crocodile3_index = idx(3);

        crocodile4 = val(4);
            crocodile4_index = idx(4);


            % take maximum values of utilities which is 1 in the beginning
            reptile(j).Itertn(i).crocodile.Position = eval(['V' num2str(crocodile_index)]);
            reptile(j).Itertn(i).crocodile.Cost = crocodile;
            reptile(j).Itertn(i).crocodile.Index =crocodile_index;
            
            reptile(j).Itertn(i).crocodile2.Position = eval(['V' num2str(crocodile2_index)]);
            reptile(j).Itertn(i).crocodile2.Cost = crocodile2;
            reptile(j).Itertn(i).crocodile2.Index = crocodile2_index;
            
            reptile(j).Itertn(i).crocodile3.Position = eval(['V' num2str(crocodile3_index)]);
            reptile(j).Itertn(i).crocodile3.Cost = crocodile3;
            reptile(j).Itertn(i).crocodile3.Index = crocodile3_index;

             reptile(j).Itertn(i).crocodile4.Position = eval(['V' num2str(crocodile4_index)]);
            reptile(j).Itertn(i).crocodile4.Cost = crocodile4;
            reptile(j).Itertn(i).crocodile4.Index = crocodile4_index;
        end

                % A$ and C$ change in each iteration; not for each robot
            disp("Iteration = " + i);
            disp("Selected candidates");
%             A1 = 2*a*rand(varSize) - a;
%             C1 = 2*rand(varSize);
%             A2 = 2*a*rand(varSize) - a;
%             C2 = 2*rand(varSize);
%             A3 = 2*a*rand(varSize) - a;
%             C3 = 2*rand(varSize);
            Dist = [0];
            
  t=1;          
     Alpha=0.1;                   % the best value 0.1
    Beta=0.005;                  % the best value 0.005
    
    ES = 2*randn - (1-t/nIter);     
     R = getOccupancy(map,reptile(j).Itertn(i).crocodile.Position) - ...
    (([(getOccupancy(map,robots(j).Itertn(i).Pose(1,:)))]))...    
    /(getOccupancy(map,reptile(j).Itertn(i).crocodile.Position)+eps);
          
 %------------------------------------------------------------------------------       
  P = Alpha + (getOccupancy(map,robots(j).Itertn(i).Pose(1,:))) - ...
            mean(getOccupancy(map,robots(j).Itertn(i).Pose(1,:))) /...
            ((getOccupancy(map,reptile(j).Itertn(i).crocodile.Position)+eps));
        

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        Eta = getOccupancy(map,reptile(j).Itertn(i).crocodile.Position).* P;
    
            for j=1:nRbt
 
   
        %if (t<nIter/4)
         %   reptile(j).Itertn(i).crocodile.X1 = (getOccupancy(map,reptile(j).Itertn(i).crocodile.Position)) -...
           %     Eta.*Beta-R .*rand;

       
        %elseif (t<2*nIter/4 && t>=nIter/4)
        
            reptile(j).Itertn(i).crocodile2.X2 = (getOccupancy(map,reptile(j).Itertn(i).crocodile2.Position)).* ...
                getOccupancy(map,robots(j).Itertn(i).Pose(1,:))* ES*rand;
        
        %elseif(t<3*(nIter/4) && t>=2*(nIter/4))

    reptile(j).Itertn(i).crocodile3.X3 =  getOccupancy(map,reptile(j).Itertn(i).crocodile3.Position)* P*rand;

      %  else 
   reptile(j).Itertn(i).crocodile4.X4 =  getOccupancy(map,reptile(j).Itertn(i).crocodile4.Position) -Eta.*eps-Eta.*rand;
 
       %    end 
        
          
          
  temp_arr = [reptile(j).Itertn(i).crocodile2.X2...
      reptile(j).Itertn(i).crocodile3.X3, reptile(j).Itertn(i).crocodile4.X4];
                [value, index] = max(temp_arr);
                reptile(j).Itertn(i).best.index = index;
                reptile(j).Itertn(i).best.value = value;

             if index == 1
                x = reptile(j).Itertn(i).crocodile.Position(1,1);
                y = reptile(j).Itertn(i).crocodile.Position(1,2);
                disp("Robot " + j +" selected crocodile position");
 
                   elseif index == 2
                       x = reptile(j).Itertn(i).crocodile2.Position(1,1);
                        y = reptile(j).Itertn(i).crocodile2.Position(1,2);
                       disp("Robot " + j + " selected crocodile2 position");
                  elseif index == 3
                       x = reptile(j).Itertn(i).crocodile3.Position(1,1);
                       y = reptile(j).Itertn(i).crocodile3.Position(1,2);
                       disp("Robot " + j + " selected crocodile3 position");
                elseif index == 4
                       x = reptile(j).Itertn(i).crocodile4.Position(1,1);
                       y = reptile(j).Itertn(i).crocodile4.Position(1,2);
                       disp("Robot " + j + " selected crocodile4 position");
             end
            
                hold on
                robots(j).Itertn(i+1).Pose = [x,y];
                insertRay(map,[x,y,0],ranges,angles,maxrange);
                hold on

            end

        disp("Parameters: " + "ES = " + ES + "a = " + a + "Beta = " + Beta);
 


        % Decreasing a parameter of 
        if i==1
           a;
        else
            a = a - (1-(t/nIter));
%            ES = 2*randn - (1-t/nIter)))
        end

        j = 1;
        hold on
        color = ['k','m','c','y'];


        for j=1:nRbt


                if j==1
                    plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(1),'marker','o');
%                     hold on
% 
%                 elseif j==2
%                     plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(2),'marker','o');
%                     hold on
% 
%                 elseif j==3
%                     plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(3),'marker','o');
%                     hold on
%             elseif j==4
%                     plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),...
%                     'color',color(4),'marker','+');
%             hold on
                end
        end

          for j=1:nRbt
            for row = 1:21
                for col = 1:21
                    gridmap_arr(j).Itertn(i).map(row,col) = getOccupancy(map,[row-1,col-1]);

                end
            end 
          end

    %     pause(0.5)
    %     hold on
        drawnow

        show(map)
        grid on
        title(['Iteration = ', num2str(i)]);

        hold on
    %     drawnow 



    end


% catch
%    warning("When neighbor cells are occupied by obstacles and another robot, this error appears. Please, run the simulation again! "); 
%    
% end

%% compute coverage bound after the run
Utility_total = 219;
Utility_covered = 0;
ranges = 9:20;
for row = 2:bound-2
    for column = 2:bound-2
        if row == 10 && (column == ranges(1) || column == ranges(2) || column == ranges(3) || column == ranges(4) || column == ranges(5) || ...
                column == ranges(6) || column == ranges(7) || column == ranges(8) || column == ranges(9) || column == ranges(10) || ...
                column == ranges(11) || column == ranges(12))
            Utility_covered;
        elseif row == 9 && (column == ranges(2) || column == ranges(3) || column == ranges(4) || column == ranges(5) || ...
                column == ranges(6) || column == ranges(7) || column == ranges(8) || column == ranges(9) || column == ranges(10) || ...
                column == ranges(11) || column == ranges(12))
            Utility_covered;
        elseif row == 11 && (column == ranges(2) || column == ranges(3) || column == ranges(4) || column == ranges(5) || ...
                column == ranges(6) || column == ranges(7) || column == ranges(8) || column == ranges(9) || column == ranges(10) ||...
                column == ranges(11) || column == ranges(12))       
            Utility_covered;
        elseif row == 7 && (column == 2 || column == 6 || column == 7 || column == 8 || column == 12 )
            Utility_covered;
        elseif row == 8 && column == 2
            Utility_covered;
        elseif row == 9 && column == 2
            Utility_covered;
        elseif row == 6 && (column == 5 || column == 6 || column == 7 || column == 8 || column == 9 || column == 11 || column == 12 || column == 13 || ...
                column == 18 || column == 19)
            Utility_covered;
        elseif row == 5 && (column == 6 || column == 7 || column == 8 || column == 9 || column == 12 || column == 17 || column == 18 || column == 19)
            Utility_covered;
        elseif row == 4 && (column == 18 || column == 19)
            Utility_covered;
        elseif row == 3 && (column == 14)
            Utility_covered;
        elseif row == 2 && (column == 13 || column == 14 || column == 15)
            Utility_covered;
        elseif row == 14 && (column == 11 || column == 12 || column == 13 || column == 18 || column == 19)
            Utility_covered;
        elseif row == 15 && (column == 10 || column == 11 || column == 12 || column == 13 || column == 14 || column == 17 || column == 18 || column == 19)
            Utility_covered;
        elseif row == 16 && (column == 11 || column == 12 || column == 13 || column == 18 || column == 19)
            Utility_covered;
        else
                
   
            Utility_covered =  Utility_covered + Utility(row, column);
        end
        
    end
end 

ExploredboundPercentage = ((Utility_total-Utility_covered)/Utility_total)*100;
%%
figure(2);

show(map);


grid on
hold on
for j=1:nRbt
        for it_=1:i
    
            if j==1
                plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(1),'marker','x');
%                 hold on
% 
%             elseif j==2
%                 plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(2),'marker','p');
%                 hold on
% 
%             elseif j==3
%                 plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(3),'marker','*');
%                 hold on
%             elseif j==4
%             plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(i).Pose(1,2),...
%                 'color',color(4),'marker','+');
%             hold on
            end
        end
        
end 
% show (map2);

grid on
title(['Iteration = ', num2str(i)]);
hold on

%view(3)
% hold off
% 
% figure;
% 
% plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(1),'marker','o')

