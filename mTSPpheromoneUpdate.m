function [t] = mTSPpheromoneUpdate(m,n,t,tour,f,e,sales)
 for i=1:m 
     for j=1:n+sales-1
         dt=1/f(i); 
         t(tour(i,j),tour(i,j+1))=(1-e)*t(tour(i,j),tour(i,j+1))+dt;
     end 
 end

