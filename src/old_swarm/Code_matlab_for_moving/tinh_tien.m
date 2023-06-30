clc
close all
clear all
t=0;
x=0;y=0;
x1=15;y1=48;

figure('name','Nem xien','color','white','numbertitle','on');
hold on
grid on
xlim([-1000 1000]); ylim([-1000 1000]); 
fig_quanang = plot(x,y,'ro','MarkerSize',5,'markerfacecolor','r');
plot(x1,y1,'s','MarkerSize',5,'markerfacecolor','blue');

while x<100
x=x+0.2*0.2;
y=y+0.8*0.2;
t=t+0.02;
plot(x,y,'o','markersize',0.5,'color','k');
set(fig_quanang,'xdata',x,'ydata',y);

if x>100
    break;
end
pause(0.02)
end

