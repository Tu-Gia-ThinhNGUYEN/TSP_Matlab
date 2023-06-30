% -------------------------------------------------------------------------
%
% File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [] = DynamicWindowApproachSample()
 
close all;
clear all;
 
disp('Dynamic Window Approach sample program start!!')

x=[0 0 pi/2 0 0]';%initial state of robot [x(m),y(m),yaw(Rad),v(m/s),É÷(rad/s)]
goal=[10,10];%goal position [x(m),y(m)]
%obstacles list [x(m) y(m)]
obstacle=[0 2;
          4 2;
          4 4;
          5 4;
          5 5;
          5 6;
          5 9
          8 8
          8 9
          7 9];
      
obstacleR=0.5;%obstacles radius
global dt; dt=0.1;%rising time [s]

%Mehanical model of robot
%Maximum velocity [m/s],maximum angular velocity [rad/s],maximum acceleration [m/ss],maximum angular acceleration [rad/ss],
%Veloctity resolution [m/s],angular velocity resolution [rad/s]]
Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];

%Function evaluation parameter [heading,dist,velocity,predictDT]
evalParam=[0.1,0.2,0.1,3.0];
area=[-1 11 -1 11];%range of map [xmin xmax ymin ymax]

%Result of simulation
result.x=[];
tic;
%movcount=0;
% Main loop
for i=1:5000
    %Calculate input value by DWA
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);%Movement by motion model
    
    %Saving simulation results
    result.x=[result.x; x'];
    
    %Goal judgment
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;
    ArrowLength=0.5;%arrow length
    %robot
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    plot(result.x(:,1),result.x(:,2),'-b');hold on;
    plot(goal(1),goal(2),'*r');hold on;
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
    %Find display position
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% Get animation frame
end
figure(2)
plot(result.x(:,4));
toc
%movie2avi(mov,'movie.avi');
 

function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
%Function to calculate the input value by DWA

%Dynamic Window[vmin,vmax,É÷min,É÷max]
Vr=CalcDynamicWindow(x,model);
%Calculation of Merit Function
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%Normalization of each merit function
evalDB=NormalizeEval(evalDB);

%Calculation of final rating value
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);%Calculate the index of the input value with the highest evaluation value
u=evalDB(ind,1:2)';%Return input value with high evaluation value

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%A function that computes the evaluation value for each pass
evalDB=[];
trajDB=[];

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        %Trajectory estimation
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);
        %Calculation of each evaluation function
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        
        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];     
    end
end

function EvalDB=NormalizeEval(EvalDB)
%function to normalize evaluation values
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end

function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
%Function to create trajectory data
global dt;
time=0;
u=[vt;ot];%Input value
traj=x;%Trajectory data
while time<=evaldt
    time=time+dt;%Simulation time update
    x=f(x,u);%Transition by exercise model
    traj=[traj x];
end

function stopDist=CalcBreakingDist(vel,model)
%A function that calculates the braking distance from the current speed according to the dynamics model
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;%Braking distance calculation
    vel=vel-model(3)*dt;%Supreme Principle
end

function dist=CalcDistEval(x,ob,R)
%Function to calculate the distance evaluation value with obstacles

dist=2;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%Calculate norm error between path position and obstacle
    if dist>disttmp%find the minimum
        dist=disttmp;
    end
end

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

function Vr=CalcDynamicWindow(x,model)
%Calculate DyamicWindow from model and current state
global dt;
%Window by vehicle model
Vs=[0 model(1) -model(2) model(2)];

%Window by motion model
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

%Calculation of the final Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,É÷min,É÷max]

function x = f(x, u)
% Motion Model
global dt;
 
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];

x= F*x+B*u;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;