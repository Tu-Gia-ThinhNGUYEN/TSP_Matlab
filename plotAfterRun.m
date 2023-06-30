%% Load image map
mymap = imread('./images/mymapC304.pgm');
mymapSize = size(mymap);
limitMymapX = mymapSize(1,2);
limitMymapY = mymapSize(1,1);

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
MAP1 = getOccupancy(binMap);
tempMatrix = (ones(MAX_X,MAX_Y))*2;
MAP1 = MAP1*(-3) + tempMatrix;

%% Plot Triangle Formation with ITSP
figure
axis([1 MAX_X 1 MAX_Y]);
% spy(obsMatrix);
grid on;
hold on;
for i=1:MAX_X
    for j=1:MAX_Y
        %         if (MAP(i,j)==-1)
        %             plot(i+.5,j+.5,"Marker",".", 'color',[205/255,229/255,255/255,0.1]);
        %             hold on;
        %         end
        if (MAP1(i,j)==-1)
            plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','k');
            hold on;
        end
    end
end

for i=1:size(cityPoints,1)
    plot(cityPoints(i,1)+.5,cityPoints(i,2)+.5,'bo');
    text(cityPoints(i,1)-10,cityPoints(i,2)+10,'Station '+string(i),'Color',[18/255, 0, 255/255,0.2],'FontSize',8);
    hold on;
end

title('Triangle Formation combine with ITSP');
subtitle({['The Best Tour: ',num2str(k),' unit'] ...
    ['Tour: ',num2str(besttour(l,:))]});
xlabel('X [unit]');
ylabel('Y [unit]');

h1 = plot(leaderPath(:,1),leaderPath(:,2),'r-','LineWidth',1.5);
hold on;
h2 = plot(follower2PathNew(:,1),follower2PathNew(:,2),'g-','LineWidth',1.5);
hold on;
h3 = plot(follower3PathNew(:,1),follower3PathNew(:,2),'m-','LineWidth',1.5);
hold on;
h4 = plot(optimalPath(:,1)+0.5, optimalPath(:,2)+0.5, 'k--','LineWidth',1.5);
hold on;

qw{1} = plot(nan, 'r-','LineWidth',1.5);
qw{2} = plot(nan, 'g-','LineWidth',1.5);
qw{3} = plot(nan, 'm-','LineWidth',1.5);
qw{4} = plot(nan, 'k--','LineWidth',1.5);
legend([qw{:}], {'Leader','Follower 1','Follower 2', 'Optimal Path'}, 'location', 'northwest')
%% Plot movable path with optimal path
figure
axis([1 MAX_X 1 MAX_Y]);
% spy(obsMatrix);
grid on;
hold on;
for i=1:MAX_X
    for j=1:MAX_Y
        %         if (MAP(i,j)==-1)
        %             plot(i+.5,j+.5,"Marker",".", 'color',[205/255,229/255,255/255,0.1]);
        %             hold on;
        %         end
        if (MAP1(i,j)==-1)
            plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','k');
            hold on;
        end
    end
end

for i=1:size(cityPoints,1)
    plot(cityPoints(i,1)+.5,cityPoints(i,2)+.5,'bo');
    text(cityPoints(i,1)-10,cityPoints(i,2)+10,'Station '+string(i),'Color',[18/255, 0, 255/255,0.2],'FontSize',8);
    hold on;
end

title('The Optimal Path');
subtitle({['The Best Tour: ',num2str(k),' unit'] ...
    ['Tour: ',num2str(besttour(l,:))]});
xlabel('X [unit]');
ylabel('Y [unit]');

for i=1:size(dOptimalPath,1)
    for j=1:size(dOptimalPath,1)
        if (i~=j)
            movablePath = dOptimalPath{i,j};
            plot(movablePath(:,1)+.5, movablePath(:,2) +.5, 'g:', 'LineWidth',1);
            hold on;
        end
    end
end
plot(optimalPath(:,1)+.5,optimalPath(:,2)+.5,'r-','LineWidth',1.5);
hold on;
qw{1} = plot(nan, 'g:','LineWidth',1);
qw{2} = plot(nan, 'r-','LineWidth',1.5);
legend([qw{:}], {'Movable Path','Optimal Path'}, 'location', 'northwest')
%% Plot only movable path
figure
subplot(1,2,1);
imagesc(d);
colorbar;
title('The Distance Matrix');
subtitle({['Number of Station: ',num2str(cityPoints_count)]});
xlabel('Station Target');
ylabel('Station Start');
subplot(1,2,2);
axis([1 MAX_X 1 MAX_Y]);
% spy(obsMatrix);
grid on;
hold on;
for i=1:MAX_X
    for j=1:MAX_Y
        %         if (MAP(i,j)==-1)
        %             plot(i+.5,j+.5,"Marker",".", 'color',[205/255,229/255,255/255,0.1]);
        %             hold on;
        %         end
        if (MAP1(i,j)==-1)
            plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','k');
            hold on;
        end
    end
end

for i=1:size(cityPoints,1)
    plot(cityPoints(i,1)+.5,cityPoints(i,2)+.5,'bo');
    text(cityPoints(i,1)-10,cityPoints(i,2)+10,'Station '+string(i),'Color',[18/255, 0, 255/255,0.2],'FontSize',8);
    hold on;
end
for i=1:size(dOptimalPath,1)
    for j=1:size(dOptimalPath,1)
        if (i~=j)
            movablePath = dOptimalPath{i,j};
            plot(movablePath(:,1)+.5, movablePath(:,2) +.5, 'g:', 'LineWidth',1);
            hold on;
        end
    end
end
title('The Movable Path Between 2 Stations');
subtitle({['Number of Station: ',num2str(cityPoints_count)]});
xlabel('X [unit]');
ylabel('Y [unit]');
qw{1} = plot(nan, 'g:','LineWidth',1);
legend([qw{:}], {'Movable Path'}, 'location', 'northwest')
%% A star from station 1 to station n/2
figure
axis([1 MAX_X 1 MAX_Y]);
% spy(obsMatrix);
grid on;
hold on;
for i=1:MAX_X
    for j=1:MAX_Y
        %         if (MAP(i,j)==-1)
        %             plot(i+.5,j+.5,"Marker",".", 'color',[205/255,229/255,255/255,0.1]);
        %             hold on;
        %         end
        if (MAP1(i,j)==-1)
            plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','k');
            hold on;
        end
    end
end

plot(cityPoints(1,1)+.5,cityPoints(1,2)+.5,'bo');
text(cityPoints(1,1)-10,cityPoints(1,2)+10,'Station '+string(1),'Color',[18/255, 0, 255/255,0.2],'FontSize',8);
hold on;

plot(cityPoints(round(cityPoints_count/2),1)+.5,cityPoints(round(cityPoints_count/2),2)+.5,'bo');
text(cityPoints(round(cityPoints_count/2),1)-10,cityPoints(round(cityPoints_count/2),2)+10,'Station '+string(round(cityPoints_count/2)),'Color',[18/255, 0, 255/255,0.2],'FontSize',8);
hold on;

xTarget = cityPoints(round(cityPoints_count/2),1);
yTarget = cityPoints(round(cityPoints_count/2),2);
xStart = cityPoints(1,1);
yStart = cityPoints(1,2);

tstart=text(xStart-18,yStart+4,'Start','Color','r','FontWeight','bold');
ttarget=text(xTarget+4,yTarget+4,'Target','Color','r','FontWeight','bold');
hold on;

title('The Optimal Path Between 2 Stations');
subtitle({['The Distance of Path: ',num2str(d(1,round(cityPoints_count/2))),' unit']});
xlabel('X [unit]');
ylabel('Y [unit]');
movablePath2Stations = dOptimalPath{1,round(cityPoints_count/2)};
plot(movablePath2Stations(:,1)+.5, movablePath2Stations(:,2) +.5, 'g:', 'LineWidth',1.5);
hold on;

qw{1} = plot(nan, 'g:','LineWidth',1.5);
legend([qw{:}], {'Movable Path'}, 'location', 'northwest')