% Define the problem: cities, obstacles, and curvature constraint
num_cities = 10;
num_obstacles = 2;
curvature_constraint = 0.5;

% Generate random cities and obstacles
cities = rand(num_cities, 2);
obstacles = rand(num_obstacles, 2);

% Initialize parameters for ACO algorithm
num_ants = 50;
num_iterations = 100;
alpha = 1;
beta = 2;
rho = 0.5;
Q = 100;
pheromone = ones(num_cities, num_cities);

% Main loop of ACO algorithm
for iter = 1:num_iterations
    % Initialize ant colony
    ants = randi(num_cities, num_ants, 1);
    best_path = [];
    best_distance = inf;
    
    % Traverse the graph with each ant
    for ant = 1:num_ants
        current_city = ants(ant);
        visited_cities = current_city;
        path_distance = 0;
        
        % Move to the next city with highest probability
        while length(visited_cities) < num_cities
            unvisited_cities = setdiff(1:num_cities, visited_cities);
            probabilities = zeros(1, length(unvisited_cities));
            
            % Calculate probabilities for each unvisited city
            for i = 1:length(unvisited_cities)
                city = unvisited_cities(i);
                probability = pheromone(current_city, city)^alpha * (1 / pdist([cities(current_city,:); cities(city,:)], 'euclidean'))^beta;
                
                % Check curvature constraint and obstacle avoidance
                if length(visited_cities) >= 2
                    prev_city = visited_cities(end-1);
                    angle = atan2(cities(city,2)-cities(current_city,2), cities(city,1)-cities(current_city,1)) - atan2(cities(current_city,2)-cities(prev_city,2), cities(current_city,1)-cities(prev_city,1));
                    if abs(angle) > curvature_constraint || any(inpolygon(cities(city,1), cities(city,2), obstacles(:,1), obstacles(:,2)))
                        probability = 0;
                    end
                end
                
                probabilities(i) = probability;
            end
            
            % Move to the next city with highest probability
            probabilities = probabilities / sum(probabilities);
            current_city = randsample(unvisited_cities, 1, true, probabilities);
            visited_cities = [visited_cities current_city];
            path_distance = path_distance + pdist([cities(visited_cities(end-1),:); cities(current_city,:)], 'euclidean');
        end
        
        % Deposit pheromone along the path
        for i = 1:length(visited_cities)-1
            pheromone(visited_cities(i), visited_cities(i+1)) = pheromone(visited_cities(i), visited_cities(i+1)) + Q/path_distance;
            pheromone(visited_cities(i+1), visited_cities(i)) = pheromone(visited_cities(i+1), visited_cities(i)) + Q/path_distance;
        end
        
        % Update best path found by the ants
        if path_distance < best_distance
            best_path = visited_cities;
            best_distance = path_distance;
        end
    end
    
    % Update pheromone levels
    pheromone = (1-rho)*pheromone;
    for i = 1:num_cities-1
        for j = i+1:num_cities
            pheromone(i,j) = pheromone(i,j) + rho*pheromone(i,j);
            pheromone(j,i) = pheromone(i,j);
        end
    end
end

% Plot the best path found
best_path = [best_path best_path(1)];
plot(cities(:,1), cities(:,2), 'bo', 'MarkerSize', 10, 'LineWidth', 1.5);
hold on;
plot(cities(best_path,1), cities(best_path,2), 'r-', 'LineWidth', 1.5);
hold off;
