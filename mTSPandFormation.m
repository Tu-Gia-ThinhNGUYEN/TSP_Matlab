clear all; clc;
%% Load image map
mymap = imread('./images/mymapC304.pgm');
mymapSize = size(mymap);
limitMymapX = mymapSize(1,2);
limitMymapY = mymapSize(1,1);
figure('Name','Load Map');

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

%% Exchange map matrix value
% -1: obstacle
%  2: free
%  0: target point
%  1: start point
MAP = getOccupancy(binMap);
tempMatrix = (ones(MAX_X,MAX_Y))*2;
MAP = MAP*(-3) + tempMatrix;

axis([1 MAX_X 1 MAX_Y]);
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
    text(xval-10,yval+10,'Station '+string(n),'Color',[18/255, 0, 255/255,0.2],'FontSize',8);
    hold on;
    n=n+1;
 end%End of While loop
cityPoints_count=size(cityPoints,1);
%% Add all points to MAP matrix as obstacles
for i=1:cityPoints_count
    MAP(cityPoints(i,1),cityPoints(i,2))=-1;
end
%% FIND MOVABLE PATH AND CALCULATE DISTANCE BY A*
% Start A* aglorithm
% Lists used for aglorithm
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

v = 3; % Volume of each salesman, each salesman can bring goods and move to maximum v stations
depot = 6;

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

sales = fix((n-1)/v)
if mod(n-1,v)~=0
    sales = sales + 1;
end
%% Ant Colony Optimization Algorithm
% The main part of the script is the implementation of the ACO algorithm, which is executed for "miter" iterations.
% The algorithm consists of four steps:
besttourToDisplay = [];
for i=1:miter
    % Step 1: Initializing the ants: For each ant, a random starting city is selected.
    for j=1:m
%         city(j,1)=fix(1+rand*(n-1));
        city(j,1)=depot;
    end
    % Step 2:  Constructing solutions: Each ant constructs a solution by moving from its current city to the next city 
    % based on the pheromone level and the heuristic information.
    [tour] = mTSPantTour(city, m, n, h, t, alpha, beta, sales, v);
    tour=horzcat(tour,tour(:,1));
    % Step 3: Updating pheromone levels: After all ants have constructed a solution, 
    % the pheromone level on each edge is updated based on the distance of the tour.
    [distance] = mTSPcalculateDistance(m, n, d, tour, sales);
    [t] = mTSPpheromoneUpdate(m, n, t, tour, distance, p, sales);
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

generalPath = besttour(l,:)
pathOfmTSP = {};
temp = [];
j=0;
index = 1;
for i=1:size(generalPath,2)-1
    temp(1,i-j) = generalPath(1,i);
    if generalPath(1,i+1) == depot
        temp(1,i-j+1) = depot;
        pathOfmTSP{1,index} = temp;
        temp = [];
        j = i;
        index = index + 1;
    end
end
pathObtained = fix((n-1)/sales)+2;
colour = ['-r';'-m';'-b'];
index=1;
for i=1:size(pathOfmTSP,2)
    for j=1:size(pathOfmTSP{1,i},2)-1
        pathTemp = dOptimalPath{pathOfmTSP{1,i}(1,j),pathOfmTSP{1,i}(1,j+1)};
        plot(pathTemp(:,1)+.5,pathTemp(:,2)+.5,colour(index,:),'LineWidth',2,'DisplayName','Optimal Path');
        hold on;
    end
    index = index + 1;
end
title('Solve IMTSP with ACO Algorithm');
subtitle({['The Best Tour: ',num2str(k),' unit']});
xlabel('X [unit]');
ylabel('Y [unit]');

qw{1} = plot(nan, 'r-','LineWidth',1.5);
qw{2} = plot(nan, 'm-','LineWidth',1.5);
qw{3} = plot(nan, 'b-','LineWidth',1.5);
legend([qw{:}], {'Robot 1','Robot 2','Robot 3'}, 'location', 'northwest')