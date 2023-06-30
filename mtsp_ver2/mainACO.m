clear all;
clc;
%% Input parameters
% The variables "miter" and "m" are the number of iterations and the number of ants, respectively.
miter = 100; % The number of iterations
m = 100; % The number of ants

v = 20; % Volume of each salesman, each salesman can bring goods and move to maximum v stations
depot = 10;


%% Parameters of the algorithm
% The variables "p", "alpha", and "beta" are the pheromone evaporation rate, the heuristic information influence, 
% and the pheromone influence, respectively.
p = .15;    % The pheromone evaporation rate
alpha = 2;  % the pheromone influence
beta = 4;   % The heuristic information influence

%% Loading data
% The script reads in the coordinates of the cities from a text file using the "fscanf" function. 
% The first column contains the x-coordinates, and the second column contains the y-coordinates. 
% These coordinates are stored in arrays "x" and "y."
file = fopen('E:\Programming\Swarm\Input\Input100.txt', 'r');
s = fscanf(file, '%d');
for i = 1:(length(s)/2)
    x(i) = [s(i*2-1)];
    y(i) = [s(i*2)];
end;
n = length(x); % The number of cities
t = 0.0001*ones(n); % Khoi tao ma tran mui

sales = fix((n-1)/v)
if mod(n-1,v)~=0
    sales = sales + 1;
end
%% Plotting the cities
% The script plots the cities on a two-dimensional plane using the "plot" function.
% The x and y coordinates are passed as arguments, along with some formatting options.

%% Calculating distances
% The script calculates the distance between each pair of cities using the Euclidean distance formula.
% The distances are stored in a matrix "d."
for i=1:n
    for j=1:n
        d(i,j)=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
    end
end
%% Creating the heuristic matrix
% The script creates a matrix "h" that contains the heuristic information for each pair of cities.
% The heuristic value is defined as 1/distance.
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
for i=1:miter
    % Step 1: Initializing the ants: For each ant, a random starting city is selected.
    for j=1:m
        % city(j,1)=fix(1+rand*(n-1));
        city(j,1)=depot;
    end
    % Step 2:  Constructing solutions: Each ant constructs a solution by moving from its current city to the next city 
    % based on the pheromone level and the heuristic information.
    [tour] = antTour(city, m, n, h, t, alpha, beta,sales,v);
    tour=horzcat(tour,tour(:,1));
    % Step 3: Updating pheromone levels: After all ants have constructed a solution, 
    % the pheromone level on each edge is updated based on the distance of the tour.
    [distance] = calculateDistance(m, n, d, tour,sales);
    [t] = pheromoneUpdate(m, n, t, tour, distance, p,sales);
    % Step 4: Evaluating the best solution: The best solution found in the current iteration is recorded.
    [bestSolution(i),best_index]=min(distance);
    besttour(i,:)=tour(best_index,:);
    disp(['Progress: ',num2str(i),'%    Current Distance: ',num2str(min(distance))]);
end
%% Plotting the best tour
figure
axis([0 1000 0 1000]);
hold on;
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
pathOfmTSP
pathObtained = fix((n-1)/sales)+2;
for i=1:size(pathOfmTSP,2)
    for j=1:size(pathOfmTSP{1,i},2)
        X(j) = x(pathOfmTSP{1,i}(1,j));
        Y(j) = y(pathOfmTSP{1,i}(1,j));
        index = index + 1;
    end
    plot(X,Y);
    hold on;
end

for i=1:size(x,2)
    plot(x,y,"bo");
    hold on;
end

% for i=1:n
%     text(x(i)-.4,y(i)+0.5,['Station ',num2str(i)]);
% end
title('Solve MTSP with ACO Algorithm');
subtitle({['The Best Tour: ',num2str(k),' unit']});
% subtitle({['The Best Tour: ',num2str(k),' unit'] ...
%     ['Tour: ',num2str(besttour(l,:))]});
xlabel('X [unit]');
ylabel('Y [unit]');

grid on;

% for i=1:n+1+sales-1
%     X(i)=x(besttour(l,i));
%     Y(i)=y(besttour(l,i));
% end
% plot(X,Y,'--ro', 'MarkerEdgeColor','b');
% for i=1:n+sales-1
%     text(X(i)-.4,Y(i)+0.5,['Station ',num2str(besttour(l,i))]);
% end
% title('Solve TSP with ACO Algorithm');
% subtitle({['The Best Tour: ',num2str(k),' unit'] ...
%     ['Tour: ',num2str(besttour(l,:))]});
% xlabel('X [unit]');
% ylabel('Y [unit]');
% 
% grid on;
% disp(['Duong ngan nhat (ACO) = ',num2str(k)]);