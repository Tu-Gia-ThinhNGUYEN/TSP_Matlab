clear
clc

%% Input parameters
miter = 100; % The number of iterations
m = 3; % The number of ants/salesmen

%% Parameters of the algorithm
p = .15; % The pheromone evaporation rate
alpha = 2; % The pheromone influence
beta = 4; % The heuristic information influence

%% Loading data
file = fopen('E:\Programming\Swarm\Input\Input10.txt', 'r');
s = fscanf(file, '%d');
for i = 1:(length(s)/2)
    x(i) = s(i*2-1);
    y(i) = s(i*2);
end
n = length(x); % The number of cities
t = 0.0001*ones(n); % Initialization of the pheromone matrix

%% Plotting the cities
subplot(1,2,1);
plot(x, y, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
xlabel('x');
ylabel('y');
for i = 1:n
    text(x(i)-.4, y(i)+2, ['TP', num2str(i)]);
end
for i = 1:n
    text(x(i)-.4, y(i)-2, ['(', num2str(x(i)), ',', num2str(y(i)), ')']);
end
title('City Coordinates');
xlabel('x');
ylabel('y');
grid on;

%% Calculating distances
d = zeros(n);
for i = 1:n
    for j = 1:n
        d(i, j) = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
    end
end

%% Creating the heuristic matrix
h = zeros(n);
for i = 1:n
    for j = 1:n
        if d(i, j) == 0
            h(i, j) = 0;
        else
            h(i, j) = 1 / d(i, j);
        end
    end
end

%% Ant Colony Optimization Algorithm
bestSolution = zeros(miter, 1);
bestTour = zeros(miter, n+1);
for iter = 1:miter
    % Step 1: Initializing the ants
    city = zeros(m, 1);
    for s = 1:m
        city(s) = fix(1 + rand*(n-1));
    end
    
    % Step 2: Constructing solutions
    tour = zeros(m, n);
    for s = 1:m
        [tour(s, :)] = antTour(city(s), 1, n, h, t, alpha, beta);
    end
    
    % Step 3: Updating pheromone levels
    distance = zeros(m, 1);
    for s = 1:m
        % tour(s, :) = horzcat(tour(s, :), tour(s, 1));
        tour=horzcat(tour,tour(:,1));
        distance(s) = calculateDistance(1, n, d, tour(s, :));
    end
    t = pheromoneUpdate(1, n, t, tour, distance, p);
    
    % Step 4: Evaluating the best solution
    [bestSolution(iter), best_index] = min(distance);
    % bestTour(iter, :) = tour(best_index, :);
    besttour(i,:)=tour(best_index,:);
    
    disp(['Progress: ', num2str(iter), '%    Current Distance: ', num2str(min(distance))]);
end

%% Plotting the best tour
[minDistance, bestIter] = min(bestSolution);
bestTour = bestTour(bestIter, :);
% X = x(bestTour);
% Y = y(bestTour);
k = minDistance;
l = bestIter;
for i=1:n+1
    X(i)=x(besttour(l,i));
    Y(i)=y(besttour(l,i));
    disp(['Best Tour: ',num2str(x)]);
end

subplot(1,2,2);
plot(X, Y, '--ro', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
xlabel('x');
ylabel('y');
for i = 1:n
    text(X(i)-.4, Y(i)+2, ['TP', num2str(bestTour(i))]);
end
title(['Shortest Route (ACO) = ', num2str(minDistance)]);
grid on;
disp(['Shortest Route (ACO) = ', num2str(minDistance)]);
