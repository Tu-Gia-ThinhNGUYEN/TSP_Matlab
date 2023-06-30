% syms t;
% P=[0 0.1 1 1.8 2];
% Tp=[0 0.4 1 1.5 2];
x = optimalPath(:,1).';
y = optimalPath(:,2).';
% x = [1 2 3 5 2 5 3 2 4];
% y = [3 6 2 3 2 1 3 4 2];
thetaSpline = theta.';
vSpline = v.';
wSpline = w.';
dt = 0.01;
% t = 0:dt:(sizeOfOptimalPath(1,1)-1)*dt;
t = linspace(0,1,numel(x));

m=1000;
tt = linspace(0,1,m);
% xx = linspace(x(1),x(size(x,2)),1000);
% yy = linspace(y(1),y(size(y,2)));
xx = spline(t, x, tt);
yy = spline(t, y, tt);
theta2 = spline(t, thetaSpline, tt);
vSpline2 = spline(t, vSpline, tt);
wSpline2 = spline(t, wSpline, tt);

figure;
plot(x,y,'ro');
hold on;
plot(xx,yy, 'b', 'LineWidth', 1.5);

% q0n=0;
% for i=1:n
%     l=1;
%     for j=1:n
%         if i==j
%         else
%             l=l*(t-Tp(j))/(Tp(i)-Tp(j));
%         end
%     end
%     q0n=q0n+P(i)*l;
% end
% 
% y=zeros(201,1);
% for i=1:201
%     y(i)=subs(q0n,t,(i-1)*0.01);
% end
% t=0:0.01:2;
% plot(t,y);
% hold on;
% plot(Tp,P,'ro');