function [distance] = calculateDistance(m,n,d,tour,sales);
for i=1:m
    s=0;
    for j=1:n+sales-1
        s=s+d(tour(i,j),tour(i,j+1));
    end
    distance(i)=s;
end