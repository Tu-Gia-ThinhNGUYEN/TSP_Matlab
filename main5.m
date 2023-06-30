clear all; clc;
%% Load image map
mymap = imread('./images/mymapC304.pgm');
mymapSize = size(mymap);
limitMymapX = mymapSize(1,2);
limitMymapY = mymapSize(1,1);
figure('Name','Load Map');

% subplot(2,2,1);
% image(mymap);
% title('Original Map');
% xlabel('Width (pixel)');
% ylabel('Heigth (pixel');
% hold on;

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
% imshow(imageCropped)
% imageNorm = double(imageCropped)/255;
% imageOccupancy = 1 - imageNorm;
% map = occupancyMap(imageOccupancy,20);
% subplot(2,2,1);
% show(map);
% hold on;

%% Convert image to binary occupancy map
imageBW = imageCropped < 100;
binMap = binaryOccupancyMap(imageBW);
% subplot(2,2,2);
% show(binMap);
% grid on;
% hold on;

%% Define 2D map
% Obstacles have been calculated
obsMatrix = occupancyMatrix(binMap);
obsMatrixSize = size(obsMatrix);
%DEFINE THE 2-D MAP ARRAY
MAX_X=obsMatrixSize(1,1);
MAX_Y=obsMatrixSize(1,2);
%This array stores the coordinates of the map and the 
%Objects in each coordinate
% MAP=2*(ones(MAX_X,MAX_Y));

%% Exchange map matrix value
% -1: obstacle
%  2: free
%  0: target point
%  1: start point
MAP = getOccupancy(binMap);
tempMatrix = (ones(MAX_X,MAX_Y))*2;
MAP = MAP*(-3) + tempMatrix;

axis([1 MAX_X 1 MAX_Y]);
% spy(obsMatrix);
grid on;
hold on;
% Plot map
for i=1:MAX_X
    for j=1:MAX_Y
        if (MAP(i,j)==-1)
            plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','k');
            hold on;
        end
    end
end
% Inflate obstacles on map
inflate(binMap, 8, "grid");
MAP = getOccupancy(binMap);
tempMatrix = (ones(MAX_X,MAX_Y))*2;
MAP = MAP*(-3) + tempMatrix;

%% Choose city points
j=0;
x_val = 1;
y_val = 1;
n=1;
cityPoints=[];

pause(2);
h=msgbox('Select city points using the Left Mouse button,to select the last city use the Right button');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
but=1;
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    
    cityPoints(n,1) = xval;
    cityPoints(n,2) = yval;
    plot(xval+.5,yval+.5,'bo');
    text(xval+4,yval+4,'City '+string(n),'Color','b');
    hold on;
    n=n+1;
 end%End of While loop
cityPoints_count=size(cityPoints,1);
%% Add all points to MAP matrix as obstacles
for i=1:cityPoints_count
    MAP(cityPoints(i,1),cityPoints(i,2))=-1;
end
%% FIND MOVABLE PATH AND CALCULATE DISTANCE BY A*
% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2

% BEGIN Interactive Obstacle, Target, Start Location selection
% pause(1);
% h=msgbox('Please Select the Target using the Left Mouse button');
% uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% but=0;
% while (but ~= 1) %Repeat until the Left button is not clicked
%     [xval,yval,but]=ginput(1);
% end
% xval=floor(xval);
% yval=floor(yval);
% xTarget=xval;%X Coordinate of the Target
% yTarget=yval;%Y Coordinate of the Target
% 
% MAP(xval,yval)=0;%Initialize MAP with location of the target
% plot(xval+.5,yval+.5,'gd');
% text(xval+1,yval+.5,'Target')

% pause(2);
% h=msgbox('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button');
%   xlabel('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button','Color','blue');
% uiwait(h,10);
% if ishandle(h) == 1
%     delete(h);
% end
% while but == 1
%     [xval,yval,but] = ginput(1);
%     xval=floor(xval);
%     yval=floor(yval);
%     MAP(xval,yval)=-1;%Put on the closed list as well
%     plot(xval+.5,yval+.5,'ro');
%  end%End of While loop
 
% pause(1);
% 
% h=msgbox('Please Select the Vehicle initial position using the Left Mouse button');
% uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% but=0;
% while (but ~= 1) %Repeat until the Left button is not clicked
%     [xval,yval,but]=ginput(1);
%     xval=floor(xval);
%     yval=floor(yval);
% end
% xStart=xval;%Starting Position
% yStart=yval;%Starting Position
% MAP(xval,yval)=1;
%  plot(xval+.5,yval+.5,'bo');
%End of obstacle-Target pickup

%% Start A* aglorithm
%% Lists used for aglorithm
countLoop = 0;
m=1; % Initial index of matrix d
dOptimalPath = {};
for (startIndex=1:cityPoints_count)
    for (targetIndex=1:cityPoints_count)
        xTarget = cityPoints(targetIndex,1);
        yTarget = cityPoints(targetIndex,2);
        MAP(xTarget,yTarget)=0;
        xval = cityPoints(startIndex,1);
        yval = cityPoints(startIndex,2);
        xStart = xval;
        yStart = yval;
        MAP(xStart,yStart)=1;
        pstart=plot(xStart+.5,yStart+.5,'ro');
        tstart=text(xStart-18,yStart+4,'Start','Color','r','FontWeight','bold');
        ttarget=text(xTarget-23,yTarget+4,'Target','Color','r','FontWeight','bold');
        hold on;
        if (xTarget==xStart && yTarget==yStart)
            d(startIndex,targetIndex)=0;
            MAP(cityPoints(startIndex,1),cityPoints(startIndex,2)) = -1;
            countLoop = countLoop+1;
            percentDisplay = (countLoop*100)/(cityPoints_count*cityPoints_count);
            disp(['Path Finding: ',num2str(percentDisplay),'%']);
            delete(pstart);
            delete(tstart);
            delete(ttarget);
            continue;
        else
            OPEN=[];
            %CLOSED LIST STRUCTURE
            %--------------
            %X val | Y val |
            %--------------
            % CLOSED=zeros(MAX_VAL,2);
            CLOSED=[];
            
            %Put all obstacles on the Closed list
            k=1;%Dummy counter
            for i=1:MAX_X
                for j=1:MAX_Y
                    if(MAP(i,j) == -1)
                        CLOSED(k,1)=i; 
                        CLOSED(k,2)=j; 
                        k=k+1;
                        % plot(xval+.5,yval+.5,'ro');
                        % hold on;
                    end
                end
            end
            CLOSED_COUNT=size(CLOSED,1);
            %set the starting node as the first node
            xNode=xval;
            yNode=yval;
            OPEN_COUNT=1;
            path_cost=0;
            goal_distance=distance(xNode,yNode,xTarget,yTarget);
            OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
            OPEN(OPEN_COUNT,1)=0;
            CLOSED_COUNT=CLOSED_COUNT+1;
            CLOSED(CLOSED_COUNT,1)=xNode;
            CLOSED(CLOSED_COUNT,2)=yNode;
            NoPath=1;
            dIndex=1;
            while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
             % plot(xNode+.5,yNode+.5,'go');
             exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
             exp_count=size(exp_array,1);
             %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
             %OPEN LIST FORMAT
             %--------------------------------------------------------------------------
             %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
             %--------------------------------------------------------------------------
             %EXPANDED ARRAY FORMAT
             %--------------------------------
             %|X val |Y val ||h(n) |g(n)|f(n)|
             %--------------------------------
             for i=1:exp_count
                flag=0;
                for j=1:OPEN_COUNT
                    if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
                        OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
                        if OPEN(j,8)== exp_array(i,5)
                            %UPDATE PARENTS,gn,hn
                            OPEN(j,4)=xNode;
                            OPEN(j,5)=yNode;
                            OPEN(j,6)=exp_array(i,3);
                            OPEN(j,7)=exp_array(i,4);
                        end%End of minimum fn check
                        flag=1;
                    end%End of node check
            %         if flag == 1
            %             break;
                end%End of j for
                if flag == 0
                    OPEN_COUNT = OPEN_COUNT+1;
                    OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
                end%End of insert new element into the OPEN list
             end%End of i for
            
             %Find out the node with the smallest fn 
              index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
              if (index_min_node ~= -1)    
               %Set xNode and yNode to the node with minimum fn
               xNode=OPEN(index_min_node,2);
               yNode=OPEN(index_min_node,3);
               path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
              %Move the Node to list CLOSED
              CLOSED_COUNT=CLOSED_COUNT+1;
              CLOSED(CLOSED_COUNT,1)=xNode;
              CLOSED(CLOSED_COUNT,2)=yNode;
              OPEN(index_min_node,1)=0;
              else
                  %No path exists to the Target!!
                  NoPath=0;%Exits the loop!
              end%End of index_min_node check
            end%End of While Loop
            %Once algorithm has run The optimal path is generated by starting of at the
            %last node(if it is the target node) and then identifying its parent node
            %until it reaches the start node.This is the optimal path
            
            i=size(CLOSED,1);
            Optimal_path=[];
            xval=CLOSED(i,1);
            yval=CLOSED(i,2);
            i=1;
            Optimal_path(i,1)=xval;
            Optimal_path(i,2)=yval;
            i=i+1;
            
            if ( (xval == xTarget) && (yval == yTarget))
                inode=0;
               %Traverse OPEN and determine the parent nodes
               parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
               parent_y=OPEN(node_index(OPEN,xval,yval),5);
               
               while( parent_x ~= xStart || parent_y ~= yStart)
                       Optimal_path(i,1) = parent_x;
                       Optimal_path(i,2) = parent_y;
                       %Get the grandparents:-)
                       inode=node_index(OPEN,parent_x,parent_y);
                       parent_x=OPEN(inode,4);%node_index returns the index of the node
                       parent_y=OPEN(inode,5);
                       i=i+1;
               end
             j=size(Optimal_path,1);
             %Plot the Optimal Path!
             plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'g:','DisplayName','Movable Path');
             hold on;
             p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'ro');
             j=j-1;
             title("Movable Path Finding Process: "+string(path_cost)+" unit");
             for i=j:-1:1
              % pause(.0001);
              set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
             drawnow limitrate;
             end
            else
             pause(1);
             h=msgbox('Sorry, No path exists to the Target!','warn');
             uiwait(h,5);
            end
            d(startIndex,targetIndex)=path_cost;
%             if (m~=1)
%                 nextIndex = dIndex;
%             else
%                 nextIndex = 0;
%             end
%             for (m = 1:size(Optimal_path,1))
%                 dOptimalPath(m+nextIndex,1) = Optimal_path(m,1);
%                 dOptimalPath(m+nextIndex,2) = Optimal_path(m,2);
%             end
%             dIndex = size(dOptimalPath,1);
            dOptimalPath{startIndex,targetIndex}=Optimal_path;
%             dIndexMatrix(startIndex,targetIndex)=dIndex;
        end % End of if
        MAP(cityPoints(startIndex,1),cityPoints(startIndex,2)) = -1;
        countLoop = countLoop+1;
        percentDisplay = (countLoop*100)/(cityPoints_count*cityPoints_count);
        disp(['Path Finding: ',num2str(percentDisplay),'%']);
        delete(p);
        delete(pstart);
        delete(tstart);
        delete(ttarget);
    end % End of target loop
end % End of start loop
disp(['Path Finding: ',num2str(percentDisplay),'%']);
%% FIND OPTIMIZE PATH BY ANT COLONY OPTIMIZATION
%% Input parameters
% The variables "miter" and "m" are the number of iterations and the number of ants, respectively.
miter = 100; % The number of iterations
m = 100; % The number of ants
%% Parameters of the algorithm
% The variables "p", "alpha", and "beta" are the pheromone evaporation rate, the heuristic information influence, 
% and the pheromone influence, respectively.
p = .15;    % The pheromone evaporation rate
alpha = 2;  % the pheromone influence
beta = 4;   % The heuristic information influence
%% Creating the heuristic matrix
% The script creates a matrix "h" that contains the heuristic information for each pair of cities.
% The heuristic value is defined as 1/distance.
n = cityPoints_count; % The number of cities
t = 0.0001*ones(n); % Init heuristic matrix
h = [];
for i=1:n
    for j=1:n
        if d(i,j)==0
            h(i,j)=0;
        else
            h(i,j)=1/d(i,j);
        end
    end
end
%% Ant Colony Optimization Algorithm
% The main part of the script is the implementation of the ACO algorithm, which is executed for "miter" iterations.
% The algorithm consists of four steps:
besttourToDisplay = [];
for i=1:miter
    % Step 1: Initializing the ants: For each ant, a random starting city is selected.
    for j=1:m
%         city(j,1)=fix(1+rand*(n-1));
        city(j,1)=1;
    end
    % Step 2:  Constructing solutions: Each ant constructs a solution by moving from its current city to the next city 
    % based on the pheromone level and the heuristic information.
    [tour] = antTour(city, m, n, h, t, alpha, beta);
    tour=horzcat(tour,tour(:,1));
    % Step 3: Updating pheromone levels: After all ants have constructed a solution, 
    % the pheromone level on each edge is updated based on the distance of the tour.
    [distance] = calculateDistance(m, n, d, tour);
    [t] = pheromoneUpdate(m, n, t, tour, distance, p);
    % Step 4: Evaluating the best solution: The best solution found in the current iteration is recorded.
    [bestSolution(i),best_index]=min(distance);
    besttour(i,:)=tour(best_index,:);
    for temp=1:n
        pathTemp = dOptimalPath{besttour(i,temp),besttour(i,temp+1)};
        tempPlot(temp) = plot(pathTemp(:,1)+.5,pathTemp(:,2)+.5,'r-','LineWidth',1,'DisplayName','Optimal Path');
        hold on;
    end
    disp(['Optimize Path: ',num2str(i),'%    Current Distance: ',num2str(min(distance))]);
    besttourToDisplay(i,1) = min(distance);
    pause(.1);
    for temp=1:n
        delete(tempPlot(temp));
    end
end
% %% Plotting the best tour
[k,l]=min(bestSolution);
optimalPath = [];
for i=1:n
    pathTemp = dOptimalPath{besttour(l,i),besttour(l,i+1)};
    plot(pathTemp(:,1)+.5,pathTemp(:,2)+.5,'r-','LineWidth',2,'DisplayName','Optimal Path');
    hold on;
    invPathTemp = [];
    for j=length(pathTemp):-1:1
        invPathTemp(length(pathTemp)-j+1,:) = pathTemp(j,:);
    end
    optimalPath = cat(1,optimalPath,invPathTemp);
end
title(['The Best Tour: ',num2str(k),' unit - Tour: ',num2str(besttour(l,:))]);
xlabel('X [unit]');
ylabel('Y [unit]');
% qw{1} = plot(nan, 'g:');
% qw{2} = plot(nan, 'r-','LineWidth',2);
% legend([qw{:}], {'Movable Path','OptimalPath'}, 'location', 'northeast')

% for i=1:n+1
%     X(i)=x(besttour(l,i));
%     Y(i)=y(besttour(l,i));
%     disp(['Best Tour: ',num2str(x)]);
% end
% subplot(1,2,2);
% plot(X,Y,'--ro', 'MarkerEdgeColor','k', 'MarkerFaceColor','g','MarkerSize',10);
% xlabel('x');ylabel('y');
% for i=1:n
%     text(X(i)-.4,Y(i)+2,['TP',num2str(besttour(l,i))]);
% end
% title(['Duong ngan nhat (ACO) = ',num2str(k)]);
% grid on;
% disp(['Duong ngan nhat (ACO) = ',num2str(k)]);
% %% BRUCE FORCE SEARCH
% %create coordinate matrix
% data = cityPoints;
% % calculate pairwise distances using pdist2
% distances = d;
% 
% % display the resulting distance matrix
% edgew=distances;
% 
% %create a matrix with number of elments equal dimmention of matrix.
% % But elements of matrix ascend order (example [1,2,3,4,5] if length(x) = 5
% notS = 1:length(data);
% % generate permutations
% N = factorial(length(data)-1);
% % generate a random integer index
% r_notS= randi(length(notS));
% %delete any element in matrix
% notS(r_notS)=[];
% % p = [5*ones(N,1) perms(notS) 5*ones(N,1)] % start and end in S
% p = [r_notS*ones(N,1) perms(notS) r_notS*ones(N,1)];
% % print one case on a single line
% 
% % printcase = @(a, p, s) fprintf('%s %c%c%c%c%c%c%c : %d\n', ...
% % a, label(p(1)),label(p(2)),label(p(3)), ...
% % label(p(4)),label(p(5)),label(p(6)),label(p(7)), s);
% 
% % evaluate cost of each (feasible) permutation
% 
% mincost = 1.0e100; % bigger than any
% % %for j = 1:(N)
% 
% for j = 1:(N/2) %Ta chia 2 vì khi chạy hoán vị factorial(5) mối kết quả sẽ bị lặp lại 1 lần
%     C = 0; % C = cost of path (or -1)
%     for k = 1:length(data)
%         w = edgew(p(j,k),p(j,k+1));
%         if w <= 0
%             C = -100;
%             break
%         else
%             C = C + w;
%             end
%         end
%         % an itinerary calculation is finished
%         if (C > 0) % report if feasible; update optimal
%         %printcase('feasible ',p(j,:),C)
%                                         %printcase = @(a, p, s) fprintf('%s %c%c%c%c%c%c%c : %d\n', ...
%                                         % a, label(p(1)),label(p(2)),label(p(3)), ...
%                                         % label(p(4)),label(p(5)),label(p(6)),label(p(7)), s);
%         if C < mincost
%             mincost = C; %best tour
%             minp = p(j,:);
%         end
%     end
% end
% %printcase('optimal ',minp,mincost)
% disp(['Optimal Path (BruceForce): ',num2str(mincost)]);
%% FORMATON CONTROL
% %% Change Optimal Path resolution
% optimalPath2(1,:)=optimalPath(1,:);
% k=2;
% for i=2:sizeOfOptimalPath(1,1)
%     if (optimalPath(i,1) ~= optimalPath(i-1,1))&&(optimalPath(i,2) ~= optimalPath(i-1,2))
%         optimalPath2(k,:)=optimalPath(i,:);
%         k=k+1;
%     end
% end
%% Initial parameter
% optimalPath = [];
leaderPath = [];
follower2Path = [];
follower3Path = [];
% invPathTemp = [];
% for i=1:n
%     pathTemp = dOptimalPath{besttour(l,i),besttour(l,i+1)};
%     for j=length(pathTemp):-1:1
%         invPathTemp(length(pathTemp)-j+1,:) = pathTemp(j,:);
%     end
%     optimalPath = cat(1,optimalPath,invPathTemp);
% end
% optimalPath = PathSmoothing(optimalPath);
sizeOfOptimalPath = size(optimalPath);
initX1 = optimalPath(1,1)+.5+5;
initY1 = optimalPath(1,2)+.5;
v1_init = 0;
a1_init = 0;
w1_init = 0;
alp1_init = 0;
dt = 0.01;

initX2 = initX1+5;
initY2 = initY1+5;
initX3 = initX1+5;
initY3 = initY1-5;
targetPoint1 = [optimalPath(1,1)+.5,optimalPath(1,2)+.5];


%%%
theta_x1 = atan2(optimalPath(1,2)+.5-initY1,optimalPath(1,1)+.5-initX1);
if theta_x1<0 %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(-theta_x1/(2*pi));
    theta_x1 = 2*(k13+1)*pi +theta_x1;
end
if theta_x1>(2*pi) %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(theta_x1/(2*pi));
    theta_x1 = theta_x1-2*(k13+1)*pi;
end
%%%%
startPoint1 = [initX1;initY1;theta_x1];

startPoint2 = [initX2;initY2;startPoint1(3,1)];
startPoint3 = [initX3;initY3;startPoint1(3,1)];
%% Calculate velocity and angular velocity for leader
t=(0:1:sizeOfOptimalPath(1,1)-1);
vx = gradient(optimalPath(:,1),t);
vy = gradient(optimalPath(:,2),t);
v = sqrt(vx.^2+vy.^2);

t_temp = t.';
dx = gradient(optimalPath(:,1));
dy = gradient(optimalPath(:,2));
theta = atan2(dy,dx);
dtheta = gradient(theta);
w = dtheta ./ gradient(t_temp);

%% First run
vw_dt = [0.04;0];
warning('off','all');
sim("formationControl.slx");
leaderPath = cat(1,leaderPath,test_x1);
follower2Path = cat(3,follower2Path,test_x2);
follower3Path = cat(3,follower3Path,test_x3);
%% Run loop

for i=2:sizeOfOptimalPath(1,1)
    vw_dt(1,1) = v(i-1);
    vw_dt(2,1) = w(i-1);
    
    currentX1 = test_x1(size(test_x1,1),1);
    currentY1 = test_x1(size(test_x1,1),2);

    currentX2 = test_x2(1,1,size(test_x2,3));
    currentY2 = test_x2(2,1,size(test_x2,3));
    currentTeta2 = test_x2(3,1,size(test_x2,3));

    currentX3 = test_x3(1,1,size(test_x3,3));
    currentY3 = test_x3(2,1,size(test_x3,3));
    currentTeta3 = test_x3(3,1,size(test_x3,3));

   
    %
    %%%
theta_x1 = atan2(optimalPath(i,2)+.5-currentY1,optimalPath(i,1)+.5-currentX1);
if theta_x1<0 %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(-theta_x1/(2*pi));
    theta_x1 = 2*(k13+1)*pi +theta_x1;
end
if theta_x1>(2*pi) %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(theta_x1/(2*pi));
    theta_x1 = theta_x1-2*(k13+1)*pi;
end
 startPoint1 = [currentX1;currentY1;theta_x1];

%%%%

    %
    startPoint1temp = startPoint1.';
    startPoint2 = [currentX2;currentY2;currentTeta2];
    startPoint3 = [currentX3;currentY3;currentTeta3];

    targetPoint1(1,1) = optimalPath(i,1)+.5;
    targetPoint1(1,2) = optimalPath(i,2)+.5;
    test_x1 = [];
    test_x2 = [];
    test_x3 = [];
    sim("formationControl.slx");
    leaderPath = cat(1,leaderPath,test_x1);
    follower2Path = cat(3,follower2Path,test_x2);
    follower3Path = cat(3,follower3Path,test_x3);
    disp(['Formating Calculation: ',num2str(i*100/sizeOfOptimalPath(1,1)),'% Target: ',num2str(targetPoint1(1,:)), ' Start: ',num2str(startPoint1temp(1,:))]);
end
%% Convert data
disp('Please wait for converting data...');
for i=1:length(follower2Path)
    follower2PathNew(i,1) = follower2Path(1,1,i);
    follower2PathNew(i,2) = follower2Path(2,1,i);
    follower2PathNew(i,3) = follower2Path(3,1,i);
end
for i=1:length(follower3Path)
    follower3PathNew(i,1) = follower3Path(1,1,i);
    follower3PathNew(i,2) = follower3Path(2,1,i);
    follower3PathNew(i,3) = follower3Path(3,1,i);
end
%% Animation
j=size(leaderPath,1);
plot(leaderPath(:,1),leaderPath(:,2),'g-','LineWidth',1);
hold on;
x1 = plot(leaderPath(1,1),leaderPath(1,2),'go');
hold on;
plot(follower2PathNew(:,1),follower2PathNew(:,2),'c-','LineWidth',1);
hold on;
x2 = plot(follower2PathNew(1,1),follower2PathNew(1,2),'co');
hold on;
plot(follower3PathNew(:,1),follower3PathNew(:,2),'m-','LineWidth',1);
hold on;
x3 = plot(follower3PathNew(1,1),follower3PathNew(1,2),'mo');
hold on;
disp('Animation start!');
for i=1:j
              set(x1,'XData',leaderPath(i,1),'YData',leaderPath(i,2));
              set(x2,'XData',follower2PathNew(i,1),'YData',follower2PathNew(i,2));
              set(x3,'XData',follower3PathNew(i,1),'YData',follower3PathNew(i,2));
             drawnow limitrate;
             disp(num2str(i));
end