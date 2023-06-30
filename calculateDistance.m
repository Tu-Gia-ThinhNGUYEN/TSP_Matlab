function [distance] = calculateDistance(m,n,d,tour);
for i=1:m
    s=0;
    for j=1:n
        s=s+d(tour(i,j),tour(i,j+1));
    end
    distance(i)=s;
end