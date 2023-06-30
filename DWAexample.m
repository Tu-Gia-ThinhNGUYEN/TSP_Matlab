 
disp('Dynamic Window Approach sample program start!!');
%% Load image map
mymap = imread('./images/mymapC304.pgm');
mymapSize = size(mymap);
limitMymapX = mymapSize(1,2);
limitMymapY = mymapSize(1,1);
%% Crop image to fit map
expandZone = 20;

for i=1:limitMymapY
    for j=1:limitMymapX
        if(mymap(i,j)==0)
            limitBot = i+expandZone;
            if (limitBot>limitMymapY)
                limitBot = limitMymapY;
            end
            break;
        end
    end
end
for i=limitMymapY:-1:1
    for j=1:limitMymapX
        if(mymap(i,j)==0)
            limitTop = i-expandZone;
            if (limitTop<=0)
                limitTop = 1;
            end
            break;
        end
    end
end
for i=1:limitMymapX
    for j=1:limitMymapY
        if(mymap(j,i)==0)
            limitRight = i+expandZone;
            if (limitRight>limitMymapX)
                limitRight = limitMymapX;
            end
            break;
        end
    end
end
for i=limitMymapX:-1:1
    for j=1:limitMymapY
        if(mymap(j,i)==0)
            limitLeft = i-expandZone;
            if (limitLeft<=0)
                limitLeft = 1;
            end
            break;
        end
    end
end
imageCropped = mymap(limitTop:limitBot,limitLeft:limitRight);
%% Convert image to binary occupancy map
imageBW = imageCropped < 100;
binMap = binaryOccupancyMap(imageBW);
%% Define 2D map
% Obstacles have been calculated
obsMatrix = occupancyMatrix(binMap);
obsMatrixSize = size(obsMatrix);
%DEFINE THE 2-D MAP ARRAY
MAX_X=obsMatrixSize(1,1);
MAX_Y=obsMatrixSize(1,2);
MAP = getOccupancy(binMap);
%% DWA
x=[30 30 pi/2 0 0]';%initial state of robot [x(m),y(m),yaw(Rad),v(m/s),ƒÖ(rad/s)]
goal=[105,120];%goal position [x(m),y(m)]
%obstacles list [x(m) y(m)]
obstacle = [];
k=1;
for i=1:MAX_X
    for j=1:MAX_Y
        if (MAP(i,j)==1)
            obstacle(k,1) = i;
            obstacle(k,2) = j;
            k=k+1;
        end
    end
end
      
obstacleR=0.1;%obstacles radius
global dt; dt=0.01;%rising time [s]

%Mehanical model of robot
%Maximum velocity [m/s],maximum angular velocity [rad/s],maximum acceleration [m/ss],maximum angular acceleration [rad/ss],
%Veloctity resolution [m/s],angular velocity resolution [rad/s]]
Kinematic=[0.1,toRadian(56.55),0.38,toRadian(56.5),0.01,toRadian(1)];

%Function evaluation parameter [heading,dist,velocity,predictDT]
evalParam=[0.1,0.2,0.1,3.0];
area=[0 241 0 213];%range of map [xmin xmax ymin ymax]

%Result of simulation
result.x=[];
tic;
%movcount=0;
% Main loop
for i=1:5000
    %Calculate input value by DWA
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);%Movement by motion model
    
    %Saving simulation results
    result.x=[result.x; x'];
    
    %Goal judgment
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;
    ArrowLength=5.5;%arrow length
    %robot
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on; %Dynamic window
    plot(result.x(:,1),result.x(:,2),'-b');hold on; % x and y of robot
    plot(goal(1),goal(2),'*r');hold on; % goal
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on; % obstacles
    %Find display position
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% Get animation frame
end
figure(2)
plot(result.x(:,4)); % velocity of robot
toc
%movie2avi(mov,'movie.avi');