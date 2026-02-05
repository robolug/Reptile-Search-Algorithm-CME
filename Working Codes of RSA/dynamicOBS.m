clc;clear;
%create a list of starting locations for n obstacles
n=10;
timeSteps=20;

obstacleStartLocation=100.*rand(n,2);
obstacleStartLocation=ceil(obstacleStartLocation);

pathX=90:-2:90-(2*(timeSteps-1));
pathY=90:-2:90-(2*(timeSteps-1));
path=[pathX' pathY'];

for i=1:timeSteps
    map = occupancyMap(zeros(100,100));%clear map
    updateOccupancy(map,obstacleStartLocation,ones(10,1));%add obstacles to map
    updateOccupancy(map,path(i,:),1);%add main path point for current time step
    inflate(map,0.5);%make point bigger as per size of obstacles
    show(map);
    pause(0.1);
    
    %update path of other obstacles randomly. I am just moving them
    %diagonally upwards in this example. please change it as per your
    %requiremnt 
    for j=1:n
        obstacleStartLocation(j,1)=obstacleStartLocation(j,1)+1;
        obstacleStartLocation(j,2)=obstacleStartLocation(j,2)+1;
    end
end