%% FORMATON CONTROL
% %% Change Optimal Path resolution
% optimalPath2(1,:)=optimalPath(1,:);
% k=2;
% for i=2:sizeOfOptimalPath(1,1)
%     if (optimalPath(i,1) ~= optimalPath(i-1,1))&&(optimalPath(i,2) ~= optimalPath(i-1,2))
%         optimalPath2(k,:)=optimalPath(i,:);
%         k=k+1;
%     end
% end
%% Initial parameter
% optimalPath = [];
leaderPath = [];
follower2Path = [];
follower3Path = [];
% invPathTemp = [];
% for i=1:n
%     pathTemp = dOptimalPath{besttour(l,i),besttour(l,i+1)};
%     for j=length(pathTemp):-1:1
%         invPathTemp(length(pathTemp)-j+1,:) = pathTemp(j,:);
%     end
%     optimalPath = cat(1,optimalPath,invPathTemp);
% end
% optimalPath = PathSmoothing(optimalPath);
sizeOfOptimalPath = size(optimalPath);
initX1 = optimalPath(1,1)+.5+5;
initY1 = optimalPath(1,2)+.5;
v1_init = 0;
a1_init = 0;
w1_init = 0;
alp1_init = 0;
dt = 0.01;

initX2 = initX1+5;
initY2 = initY1+5;
initX3 = initX1+5;
initY3 = initY1-5;
targetPoint1 = [optimalPath(1,1)+.5,optimalPath(1,2)+.5];


%%%
theta_x1 = atan2(optimalPath(1,2)+.5-initY1,optimalPath(1,1)+.5-initX1);
if theta_x1<0 %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(-theta_x1/(2*pi));
    theta_x1 = 2*(k13+1)*pi +theta_x1;
end
if theta_x1>(2*pi) %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(theta_x1/(2*pi));
    theta_x1 = theta_x1-2*(k13+1)*pi;
end
%%%%
startPoint1 = [initX1;initY1;theta_x1];

startPoint2 = [initX2;initY2;startPoint1(3,1)];
startPoint3 = [initX3;initY3;startPoint1(3,1)];
%% Calculate velocity and angular velocity for leader
t=(0:1:sizeOfOptimalPath(1,1)-1);
vx = gradient(optimalPath(:,1),t);
vy = gradient(optimalPath(:,2),t);
v = sqrt(vx.^2+vy.^2);

t_temp = t.';
dx = gradient(optimalPath(:,1));
dy = gradient(optimalPath(:,2));
theta = atan2(dy,dx);
dtheta = gradient(theta);
w = dtheta ./ gradient(t_temp);

%% First run
vw_dt = [0.04;0];
warning('off','all');
sim("formationControl.slx");
leaderPath = cat(1,leaderPath,test_x1);
follower2Path = cat(3,follower2Path,test_x2);
follower3Path = cat(3,follower3Path,test_x3);
%% Run loop

for i=2:sizeOfOptimalPath(1,1)
    vw_dt(1,1) = v(i-1);
    vw_dt(2,1) = w(i-1);
    
    currentX1 = test_x1(size(test_x1,1),1);
    currentY1 = test_x1(size(test_x1,1),2);

    currentX2 = test_x2(1,1,size(test_x2,3));
    currentY2 = test_x2(2,1,size(test_x2,3));
    currentTeta2 = test_x2(3,1,size(test_x2,3));

    currentX3 = test_x3(1,1,size(test_x3,3));
    currentY3 = test_x3(2,1,size(test_x3,3));
    currentTeta3 = test_x3(3,1,size(test_x3,3));

   
    %
    %%%
theta_x1 = atan2(optimalPath(i,2)+.5-currentY1,optimalPath(i,1)+.5-currentX1);
if theta_x1<0 %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(-theta_x1/(2*pi));
    theta_x1 = 2*(k13+1)*pi +theta_x1;
end
if theta_x1>(2*pi) %khi mega <0 se xay ra hien tuong xoay tron
    k13=fix(theta_x1/(2*pi));
    theta_x1 = theta_x1-2*(k13+1)*pi;
end
 startPoint1 = [currentX1;currentY1;theta_x1];

%%%%

    %
    startPoint1temp = startPoint1.';
    startPoint2 = [currentX2;currentY2;currentTeta2];
    startPoint3 = [currentX3;currentY3;currentTeta3];

    targetPoint1(1,1) = optimalPath(i,1)+.5;
    targetPoint1(1,2) = optimalPath(i,2)+.5;
    test_x1 = [];
    test_x2 = [];
    test_x3 = [];
    sim("formationControl.slx");
    leaderPath = cat(1,leaderPath,test_x1);
    follower2Path = cat(3,follower2Path,test_x2);
    follower3Path = cat(3,follower3Path,test_x3);
    disp(['Formating Calculation: ',num2str(i*100/sizeOfOptimalPath(1,1)),'% Target: ',num2str(targetPoint1(1,:)), ' Start: ',num2str(startPoint1temp(1,:))]);
end
%% Convert data
disp('Please wait for converting data...');
for i=1:length(follower2Path)
    follower2PathNew(i,1) = follower2Path(1,1,i);
    follower2PathNew(i,2) = follower2Path(2,1,i);
    follower2PathNew(i,3) = follower2Path(3,1,i);
end
for i=1:length(follower3Path)
    follower3PathNew(i,1) = follower3Path(1,1,i);
    follower3PathNew(i,2) = follower3Path(2,1,i);
    follower3PathNew(i,3) = follower3Path(3,1,i);
end
%% Animation
j=size(leaderPath,1);
plot(leaderPath(:,1),leaderPath(:,2),'g-','LineWidth',1);
hold on;
x1 = plot(leaderPath(1,1),leaderPath(1,2),'go');
hold on;
plot(follower2PathNew(:,1),follower2PathNew(:,2),'c-','LineWidth',1);
hold on;
x2 = plot(follower2PathNew(1,1),follower2PathNew(1,2),'co');
hold on;
plot(follower3PathNew(:,1),follower3PathNew(:,2),'m-','LineWidth',1);
hold on;
x3 = plot(follower3PathNew(1,1),follower3PathNew(1,2),'mo');
hold on;
disp('Animation start!');
for i=1:j
              set(x1,'XData',leaderPath(i,1),'YData',leaderPath(i,2));
              set(x2,'XData',follower2PathNew(i,1),'YData',follower2PathNew(i,2));
              set(x3,'XData',follower3PathNew(i,1),'YData',follower3PathNew(i,2));
             drawnow limitrate;
             disp(num2str(i));
end