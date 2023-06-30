clear all; clc;
%% Load image map
mymap = imread('./images/mymap.pgm');
mymapSize = size(mymap);
limitMymapX = mymapSize(1,2);
limitMymapY = mymapSize(1,1);
figure('Name','Load Map');

% subplot(1,2,1);
image(mymap);
title('Original Map');
xlabel('Width [unit]');
ylabel('Heigth [unit]');
hold on;

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
figure
image(imageCropped)
% imageNorm = double(imageCropped)/255;
% imageOccupancy = 1 - imageNorm;
% map = occupancyMap(imageOccupancy,20);
% subplot(1,2,2);
title("Cropped Map")
xlabel('Width [unit]');
ylabel('Heigth [unit]');
% hold on;

%% Convert image to binary occupancy map
imageBW = imageCropped < 100;
binMap = binaryOccupancyMap(imageBW);
binMap1 = binaryOccupancyMap(imageBW);
% subplot(2,2,2);
figure
show(binMap);
grid on;
hold on;

%% Define 2D map
% Obstacles have been calculated
% obsMatrix = occupancyMatrix(binMap);
% obsMatrixSize = size(obsMatrix);
%DEFINE THE 2-D MAP ARRAY
% MAX_X=obsMatrixSize(1,1);
% MAX_Y=obsMatrixSize(1,2);
%This array stores the coordinates of the map and the 
%Objects in each coordinate
% MAP=2*(ones(MAX_X,MAX_Y));

%% Exchange map matrix value
% -1: obstacle
%  2: free
%  0: target point
%  1: start point
% axis([1 MAX_X 1 MAX_Y]);
% spy(obsMatrix);
% grid on;
% hold on;
% Plot map
% Inflate obstacles on map
% inflate(binMap, 8, "grid");
% MAP = getOccupancy(binMap);
% tempMatrix = (ones(MAX_X,MAX_Y))*2;
% MAP = MAP*(-3) + tempMatrix;
% title('Map After Processed');
% xlabel('X [unit]');
% ylabel('Y [unit]');
% Plot map
% for i=1:MAX_X
%     for j=1:MAX_Y
%         if (MAP(i,j)==-1)
%             h = plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','m');
%             hold on;
%         end
%     end
% end
% 
% MAP = getOccupancy(binMap1);
% tempMatrix = (ones(MAX_X,MAX_Y))*2;
% MAP = MAP*(-3) + tempMatrix;
% for i=1:MAX_X
%     for j=1:MAX_Y
%         if (MAP(i,j)==-1)
%             plot(i+.5,j+.5,"Marker",".", 'MarkerEdgeColor','k');
%             hold on;
%         end
%     end
% end
% map = binaryOccupancyMap(0.3,0.4,20);
% x = [0];
% y = [0.3];
% 
% robot = [1 1 0;
%          1 1 1;
%          1 1 1;
%          1 1 0];
% 
% example =[0 0 0 0;
%           0 0 1 1;
%           0 0 1 0];
% ij = world2grid(map,[x y]);
% % setOccupancy(map, ij, ones(1,1),"grid")
% setOccupancy(map, ij, example, "grid")
% % inflate(map, 5,"grid")
% % setOccupancy(map, ij, zeros(1,1),"grid")
% % figure
% % show(map)
% % grid on;
% 
% obsMatrix = occupancyMatrix(map);
% obsMatrixSize = size(obsMatrix);
% %DEFINE THE 2-D MAP ARRAY
% MAX_X=obsMatrixSize(1,1);
% MAX_Y=obsMatrixSize(1,2);
% axis([1 MAX_X 1 MAX_Y]);
% grid on;
% hold on;
% for i=1:MAX_X
%     for j=1:MAX_Y
%         if (obsMatrix(i,j)==1)
%             plot(i+.5,j+.5,"ro");
%             hold on;
%         end
%     end
% end
% grid on;