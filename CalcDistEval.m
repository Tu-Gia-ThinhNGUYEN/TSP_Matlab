function dist=CalcDistEval(x,ob,R)
%Function to calculate the distance evaluation value with obstacles

dist=2;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%Calculate norm error between path position and obstacle
    if dist>disttmp%find the minimum
        dist=disttmp;
    end
end