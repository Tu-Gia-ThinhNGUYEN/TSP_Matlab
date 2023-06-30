function heading=CalcHeadingEval(x,goal)
%A function that computes the heading's evaluation function

theta=toDegree(x(3));%Robot orientation
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));%direction of the goal

if goalTheta>theta
    targetTheta=goalTheta-theta;%Azimuth difference to goal [deg]
else
    targetTheta=theta-goalTheta;%Azimuth difference to goal [deg]
end

heading=180-targetTheta;