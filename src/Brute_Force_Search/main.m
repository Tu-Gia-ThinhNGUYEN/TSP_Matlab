clear
clc
% % TSP Setup and solve a traveling salesperson problem
% label = ['A' 'F' 'J' 'N' 'S' 'W'];
% edgew = [ -1 100 100 150 250 150; % A
% 100 -1 150 200 300 250; % F
% 100 150 -1 -1 200 200; % J
% 150 200 -1 -1 -1 -1; % N
% 250 300 200 -1 -1 -1; % S
% 150 250 200 -1 -1 -1]; % W

file = fopen('E:\Programming\Swarm\Input\Input10.txt', 'r');
s = fscanf(file, '%d');
for i = 1:(length(s)/2)
    x(i) = [s(i*2-1)];
    y(i) = [s(i*2)];
end;
n = length(x); % The number of cities
for i=1:n
    label(i,1) = 'TP'+num2str(i);
end
for i=1:n
    for j=1:n
        edgew(i,j)=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
    end
end
% generate permutations
notS = [1 2 3 4 6]; % not including S
N = factorial(5); % this many permutations of notS
p = [5*ones(N,1) perms(notS) 5*ones(N,1)]; % start and end in S
% print one case on a single line
printcase = @(a, p, s) fprintf('%s %c%c%c%c%c%c%c : %d\n', ...
a, label(p(1)),label(p(2)),label(p(3)), ...
label(p(4)),label(p(5)),label(p(6)),label(p(7)), s);
% evaluate cost of each (feasible) permutation
mincost = 1.0e100; % bigger than any
for j = 1:N
C = 0; % C = cost of path (or -1)
for k = 1:6
w = edgew(p(j,k),p(j,k+1));
if w < 0
C = -1;
break
else
C = C + w;
end
end
if (C > 0) % report if feasible; update optimal
printcase('feasible ',p(j,:),C)
if C < mincost
mincost = C;
minp = p(j,:);
end
end
end
printcase('optimal ',minp,mincost)