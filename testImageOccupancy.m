clear all; clc;
mymap = imread('./images/map_c304_2.pgm');
mymapSize = size(mymap);
limitMymapX = mymapSize(1,1);
limitMymapY = mymapSize(1,2);
imageCropped = mymap(1:384,1:384);
imageBW = imageCropped < 100;
binMap = binaryOccupancyMap(imageBW);
show(binMap);
grid on;
obsMatrix = getOccupancy(binMap)