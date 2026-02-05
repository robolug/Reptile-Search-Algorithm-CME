clc;
clear;
close all;
nRbt = 3;
nOrient = 2;
nIter = 300;
bound = 30;
d = 1; %step

ray_lngth = 1.5; % max_range value in the paper

%value delta x, delta y 
y_up = 1;
y_down = -1;
y_stay = 0;
x_right = 1;
x_left = -1;
x_stay = 0;

Vrobot = 1;

anglesClsion = [pi/2,pi/4,0,7*pi/4,3*pi/2,5*pi/4,pi,3*pi/4];
thresholdClsionDttn = 0.9; %threshold of collition detection
maxrangeClsion = 2; %maxrange to detect collition


map = robotics.OccupancyGrid(bound,bound,15); %resolution is the number of pixels in one cell; no needs to consider it below
show (map);
hold on

%%
cntx = bound/2;
cnty = bound/2;
s = cnty/0.1;
A(1,s) = cntx-1;
A(1:s) = cntx;
B = cnty:0.1:bound-0.1;
C = [A; B];
C = transpose(C);
A = A/2.5;
D = [B; A];
D = transpose(D);
% q = (bound/0.1) - 10;
q = (bound/0.1);
E(1,q) = bound-1;
E(1:q) = 0.1;
I(1,q) = bound-1;
I(1:q) = bound-0.1;
% F = 1:0.1:bound-0.1;
F = 0:0.1:bound-0.1;
H = [E;F];
H = transpose(H);
G = [I;F];
G = transpose(G);
J = [F;I];
J = transpose(J);
W = [F;E];
W = transpose(W);
d_ = 0;
% 

% H left vertical
setOccupancy(map, H, 1)
% G right vertical
setOccupancy(map, G, 1)
% J upper horizontal
setOccupancy(map, J, 1)
setOccupancy(map, W, 1)

% % 
 %  inflate(map,1)
 % show(map)
%% case 1
% Define different sizes for obstacles
block_heights = [3, 4, 2.5, 3.5]; % Different heights
block_widths = [0.5, 0.8, 1.0, 0.7]; % Different widths
block_lengths = [5, 7, 6, 4]; % Different lengths

% Vertices for a base cuboid (these will be scaled)
base_vertices = [
    0, 0, 0;    % Vertex 1
    1, 0, 0;    % Vertex 2
    1, 1, 0;    % Vertex 3
    0, 1, 0;    % Vertex 4
    0, 0, 1;    % Vertex 5
    1, 0, 1;    % Vertex 6
    1, 1, 1;    % Vertex 7
    0, 1, 1;    % Vertex 8
];

faces = [
    1, 2, 3, 4;  % Bottom face
    5, 6, 7, 8;  % Top face
    1, 2, 6, 5;  % Side face 1
    2, 3, 7, 6;  % Side face 2
    3, 4, 8, 7;  % Side face 3
    4, 1, 5, 8   % Side face 4
];

num_obstacles = 7;
map_size = [30, 30]; % Assuming the map size

% Define specific positions for ordered obstacles
block_positions = [
    5, 5;            % Lower-left corner
    map_size(1)-12, map_size(2)-12;  % Upper-right corner
    8, 5;            % Middle row
    8, 10;           % Middle row
    8, 15;           % Middle row
    5, 20;           % Right column
    20, 5;           % Left column
];

% Plot multiple blocks
for i = 1:num_obstacles
    if i == 1 % First obstacle
        block_height = block_heights(1);
        block_width = block_widths(1);
        block_length = block_lengths(1); % Specific size for the first obstacle
    elseif i == 2 % Long obstacle
        block_height = block_heights(2);
        block_width = block_widths(2);
        block_length = 10; % Longer length for the long obstacle
    else % Other obstacles
        block_height = block_heights(randi(length(block_heights)));
        block_width = block_widths(randi(length(block_widths)));
        block_length = block_lengths(randi(length(block_lengths)));
    end
end
hold on;
    % Scale the base vertices
    vertices = [
        base_vertices(:, 1) * block_length, ...
        base_vertices(:, 2) * block_width, ...
        base_vertices(:, 3) * block_height
    ];

for i = 1:size(block_positions, 1)  
    % Plot the block at the specified position
    patch('Vertices', vertices + repmat([block_positions(i, :), 0], 8, 1), 'Faces', faces, 'FaceColor', 'k', 'EdgeColor', 'k');
    hold on;

    % Set occupancy in the map for the current block
   % setOccupancy(map, block_positions(i, :), 1);

    for x = block_positions(i, 1):block_positions(i, 1) + block_length - 1
        for y = block_positions(i, 2):block_positions(i, 2) + block_width - 1
            setOccupancy(map, [x y], 1);
        end
    end


end

hold on
% Set viewpoint
view(3);

% Adjust lighting for better visibility
lighting gouraud;
light('Position', [1 1 3]);

% Inflate the map
inflate(map, 1);

% Show the map
show(map)
%% case 2
% % Define larger block dimensions
% block_width = 1; % Increased width
% block_length = 1; % Increased length
% block_height = 1; % Increased height
% 
% % Vertices for a larger block
% vertices = [
%     0, 0, 0;                   % Vertex 1
%     block_width, 0, 0;         % Vertex 2
%     block_width, block_length, 0; % Vertex 3
%     0, block_length, 0;        % Vertex 4
%     0, 0, block_height;        % Vertex 5
%     block_width, 0, block_height; % Vertex 6
%     block_width, block_length, block_height; % Vertex 7
%     0, block_length, block_height; % Vertex 8
% ];
% 
% faces = [
%     1, 2, 3, 4;  % Bottom face
%     5, 6, 7, 8;  % Top face
%     1, 2, 6, 5;  % Side face 1
%     2, 3, 7, 6;  % Side face 2
%     3, 4, 8, 7;  % Side face 3
%     4, 1, 5, 8   % Side face 4
% ];
% 
% % Define fixed positions for larger obstacles
% block_positions = [
%   %  5, 5;        % Position 1
%   %  10, 10;      % Position 2
%     20, 22;      % Position 3
%     20, 5;       % Position 4
%     5, 20;       % Position 5
%    % 5,6;
%     5,7;
%     5,8; 5,9; 5,10; 5,11; 5,12; 5,13; 5,14; 5,15;
%     5,16; 6,16; 6,16; 7,16; 8,16; 9,16; 10,16; 11,16; 12,16; 13,16;
%     13,15; 13,14; 13,13; 13,12; 13,11; 13,10; 13,09;
%     14,9; 15,9; 16,9; 17,09; 18,9; 
%    19,9; 19,10; 19,11; 19,12; 19,13; 19,14; 19,15; 19,16;
%    19,17; 20,17; 21,17; 22,17; 23,17; 24,17; 25,17;
% ];
% 
% 
% % Plot multiple larger blocks at fixed positions
% for i = 1:size(block_positions, 1)
%     % Plot the block at the specified position
%     patch('Vertices', vertices + repmat([block_positions(i, :), 0], 8, 1), 'Faces', faces, 'FaceColor', 'k', 'EdgeColor', 'k');
%     hold on;
% 
%     % Set occupancy in the map for the current block
%     for x = block_positions(i, 1):(block_positions(i, 1) + block_width - 1)
%         for y = block_positions(i, 2):(block_positions(i, 2) + block_length - 1)
%             setOccupancy(map, [x y], 1);
%         end
%     end
% end
% 
% hold on
% % Set viewpoint
% view(3);
% 
% % Adjust lighting for better visibility
% lighting gouraud;
% light('Position', [1 1 3]);
% 
% % Inflate the map
% inflate(map, 1);
% 
% % Show the map
% show(map);

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

Utility_total = 251;
% for row = 2:bound-2
%     for column = 2:bound-2
%         Utility_total =  Utility_total + Utility(row, column);
%     end
% end
% Utility_total = Utility_total - 30;

%% initialization in Itertn 1
for i=1
    for j=1:nRbt
        if j==1
            robots(j).Itertn(i).Pose = [3,5,0];
%             robots(j).Itertn(i).Pose = [6,5,0];
        end
        if j==2
            robots(j).Itertn(i).Pose = [5,4,0];
%             robots(j).Itertn(i).Pose = [7,4,0];
        end
        if j==3
            robots(j).Itertn(i).Pose = [7,4,0];
%             robots(j).Itertn(i).Pose = [8,4,0];
        end
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

        

        x = robots(j).Itertn(i).Pose(1,1);
        y = robots(j).Itertn(i).Pose(1,2);
        thi = robots(j).Itertn(i).Pose(1,3);        
%         insertRay(map,[x,y,thi],ranges,angles,maxrange); 
        hold on
        
        color = ['r','b','g'];
        if j==1
            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(1),'marker','o');
            hold on
        elseif j==2
            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(2),'marker','o');
            hold on
        elseif j==3
            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(3),'marker','o');
            hold on
        end
        show(map)
        grid on
        hold on
    end
end

       

k_ = 1;

% 
%% Main source
% 
%Cost
try
    for i=1:nIter
        val = 1;
        for j=1:nRbt       

            x = robots(j).Itertn(i).Pose(1,1);
            y = robots(j).Itertn(i).Pose(1,2);
            insertRay(map,[x,y,thi],ranges,angles,maxrange);

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
                                disp([V(j).Itertn(i).V(i_,1),V(j).Itertn(i).V(i_,2)]);
                                Vblock(j).Itertn(i).V(k_,:) = [V(j).Itertn(i).V(i_,1), V(j).Itertn(i).V(i_,2)];
                                k_ = k_ + 1;

                            end
                            if j == 3
                                   if V(3).Itertn(i).V(i_,1)==V(1).Itertn(i).V(j_,1) && V(3).Itertn(i).V(i_,2)==V(1).Itertn(i).V(j_,2)
                                       disp([V(j).Itertn(i).V(i_,1),V(j).Itertn(i).V(i_,2)]);
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
                            robots(j).Itertn(i).V8.V8cost = (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
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
    %                         robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y],'euclidean'))*(getOccupancy(map,V3));
    %                         robots(j).Itertn(i).V3.V3cost = robots(j).Itertn(i-1).Cost(1,3) + (pdist([x,y;x+1,y],'euclidean'))*0;
                            robots(j).Itertn(i).V3.V3cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V4(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V4(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y-1],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y-1],'euclidean'))*(getOccupancy(map,V4));
    %                         robots(j).Itertn(i).V4.V4cost = robots(j).Itertn(i-1).Cost(1,4) + (pdist([x,y;x+1,y-1],'euclidean'))*0;
                              robots(j).Itertn(i).V4.V4cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V5(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V5(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x,y-1],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x,y-1],'euclidean'))*(getOccupancy(map,V5));
    %                         robots(j).Itertn(i).V5.V5cost = robots(j).Itertn(i-1).Cost(1,5) + (pdist([x,y;x,y-1],'euclidean'))*0;
                            robots(j).Itertn(i).V5.V5cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V6(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V6(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x-1,y-1],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x-1,y-1],'euclidean'))*(getOccupancy(map,V6));
    %                         robots(j).Itertn(i).V6.V6cost = robots(j).Itertn(i-1).Cost(1,6) + (pdist([x,y;x-1,y-1],'euclidean'))*0;
                            robots(j).Itertn(i).V6.V6cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V7(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V7(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y],'euclidean'))*(getOccupancy(map,V7));
    %                         robots(j).Itertn(i).V7.V7cost = robots(j).Itertn(i-1).Cost(1,7) + (pdist([x,y;x-1,y],'euclidean'))*0;
                            robots(j).Itertn(i).V7.V7cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V8(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V8(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x-1,y+1],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x-1,y+1],'euclidean'))*(getOccupancy(map,V8));
    %                         robots(j).Itertn(i).V8.V8cost = robots(j).Itertn(i-1).Cost(1,8) + (pdist([x,y;x-1,y+1],'euclidean'))*0;
                            robots(j).Itertn(i).V8.V8cost = 0;
                        end
                    end

                    for cnt = 1:m               
                        if V9(1,1) == Vblock(j).Itertn(i).V(cnt,1) && V9(1,2) == Vblock(j).Itertn(i).V(cnt,2)
                            robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y],'euclidean'))*0.7;
                        else
    %                         robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y],'euclidean'))*(getOccupancy(map,V9));
    %                         robots(j).Itertn(i).V9.V9cost = robots(j).Itertn(i-1).Cost(1,9) + (pdist([x,y;x,y],'euclidean'))*0;
                              robots(j).Itertn(i).V9.V9cost = 0;
                        end
                    end

                end



                robots(j).Itertn(i).Cost = [robots(j).Itertn(i).V1.V1cost, robots(j).Itertn(i).V2.V2cost, robots(j).Itertn(i).V3.V3cost, robots(j).Itertn(i).V4.V4cost,...
                robots(j).Itertn(i).V5.V5cost, robots(j).Itertn(i).V6.V6cost, robots(j).Itertn(i).V7.V7cost, robots(j).Itertn(i).V8.V8cost, ...
                robots(j).Itertn(i).V9.V9cost];


                [robots(j).Itertn(i).MinCost.Value, robots(j).Itertn(i).MinCost.Index] = min(robots(j).Itertn(i).Cost(robots(j).Itertn(i).Cost>0));

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


            end

        end


        %%  reduce Utilities
        j = 1;
        for j=1:nRbt

            t_ = robots(j).Itertn(i).MaxUtility.Index;

            %%
            Utility(robots(j).Itertn(i).V1.V1xy(1,1), robots(j).Itertn(i).V1.V1xy(1,2)) = Utility(robots(j).Itertn(i).V1.V1xy(1,1), robots(j).Itertn(i).V1.V1xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V1.V1xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V2.V2xy(1,1), robots(j).Itertn(i).V2.V2xy(1,2)) = Utility(robots(j).Itertn(i).V2.V2xy(1,1), robots(j).Itertn(i).V2.V2xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V2.V2xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V3.V3xy(1,1), robots(j).Itertn(i).V3.V3xy(1,2)) = Utility(robots(j).Itertn(i).V3.V3xy(1,1), robots(j).Itertn(i).V3.V3xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V3.V3xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V4.V4xy(1,1), robots(j).Itertn(i).V4.V4xy(1,2)) = Utility(robots(j).Itertn(i).V4.V4xy(1,1), robots(j).Itertn(i).V4.V4xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V4.V4xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V5.V5xy(1,1), robots(j).Itertn(i).V5.V5xy(1,2)) = Utility(robots(j).Itertn(i).V5.V5xy(1,1), robots(j).Itertn(i).V5.V5xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V5.V5xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V6.V6xy(1,1), robots(j).Itertn(i).V6.V6xy(1,2)) = Utility(robots(j).Itertn(i).V6.V6xy(1,1), robots(j).Itertn(i).V6.V6xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V6.V6xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V7.V7xy(1,1), robots(j).Itertn(i).V7.V7xy(1,2)) = Utility(robots(j).Itertn(i).V7.V7xy(1,1), robots(j).Itertn(i).V7.V7xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V7.V7xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V8.V8xy(1,1), robots(j).Itertn(i).V8.V8xy(1,2)) = Utility(robots(j).Itertn(i).V8.V8xy(1,1), robots(j).Itertn(i).V8.V8xy(1,2)) - (getOccupancy(map,robots(j).Itertn(i).V8.V8xy) - ...
                          getOccupancy(map,robots(j).Itertn(i).V9.V9xy));
            Utility(robots(j).Itertn(i).V9.V9xy(1,1), robots(j).Itertn(i).V9.V9xy(1,2)) = 0;

            if t_ == 1
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V1.V1xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V1.V1xy,0];
            end

            if t_ == 2
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V2.V2xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V2.V2xy,0];
            end

            if t_ == 3
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V3.V3xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V3.V3xy,0];
            end

            if t_ == 4
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V4.V4xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V4.V4xy,0];
            end

            if t_ == 5
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V5.V5xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V5.V5xy,0];
            end

            if t_ == 6
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V6.V6xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V6.V6xy,0];
            end

            if t_ == 7
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V7.V7xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V7.V7xy,0];
            end
            if t_ == 8
                robots(j).Itertn(i).MaxUtility.Uxy = robots(j).Itertn(i).V8.V8xy;
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V8.V8xy,0];
            end
            if t_ == 9
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V9.V9xy,0];
                disp("Maximum x = " + robots(j).Itertn(i).MaxUtility.Uxy(1,1) + ", y = " + robots(j).Itertn(i).MaxUtility.Uxy(1,2));
            end

    %                  
            x = robots(j).Itertn(i+1).Pose(1,1);
            y = robots(j).Itertn(i+1).Pose(1,2);
            thi = robots(j).Itertn(i+1).Pose(1,3); 
            insertRay(map,[x,y,thi],ranges,angles,maxrange);



            hold on
            color = ['r','b','g'];
            if j==1

                plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(1),'marker','o');
                hold on

            elseif j==2
                plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(2),'marker','o');
                hold on

            elseif j==3
                plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(3),'marker','o');
                hold on

            end


            for row = 1:21
                for col = 1:21
                    gridmap_arr(j).Itertn(i).map(row,col) = getOccupancy(map,[row-1,col-1]);

                end
            end

        end




    %     pause(0.5)
        drawnow
        show(map)
        grid on
        title(['Iteration = ', num2str(i)]);
        hold on
        drawnow 



    end

catch
   warning("When neighbor cells are occupied by obstacles and another robot, this error appears. Please, run the simulation again! "); 
   
end

%% compute coverage area after the run
Utility_total = 300;
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
ExploredAreaPercentage = ((Utility_total-Utility_covered)/Utility_total)*100
%%


for j=1:nRbt
        for it_=1:i
    
            if j==1
                plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(1),'marker','o');
                hold on

            elseif j==2
                plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(2),'marker','s');
                hold on

            elseif j==3
                plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(3),'marker','d');
                hold on

            end
        end
        
end 
view(3)