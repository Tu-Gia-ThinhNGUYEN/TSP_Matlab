clc
close all
clear all
%% Khởi tạo thông số
k1=0.045;k2=0.589;k3=0.455;
% k1=2.4;k2=4.7;k3=0.53;
% k1=0.45;k2=1.2;k3=0.6;
%% Khởi tạo bản vẽ
figure('name','Chuyển động','color','white','numbertitle','on');
hold on
grid on

xlim([-50 50]); ylim([-50 50]);
%% Position of vehicles
% Xe số 1
x_1=0;y_1=0;theta_1=(10/180)*pi;
fig_quanang = plot(x_1,y_1,'ro','MarkerSize',5,'markerfacecolor','r');
% Khởi tạo tạo độ điểm target 
x_target=10;y_target=15;theta_target=(-27/180)*pi;
v_target=0.05;w_target=0.05;

plot(x_target,y_target,'s','MarkerSize',4,'markerfacecolor','r');

% Thông số xe 
R =0.03255;
L =0.11;
while ((abs(x_1)<=abs(x_target))&&(abs(y_1)<=abs(y_target)))
% while ( (x_1~=x_target) && (y_1~=y_target)&&(theta_1~=theta_target))
           A  =[cos(theta_1) sin(theta_1) 0;
               -sin(theta_1) cos(theta_1) 0;
               0             0            1];
           B  =[x_target-x_1;
                y_target-y_1;
                theta_target-theta_1];
           error=A*B;

           Vc=[ v_target*cos(error(3,1)) + k1*error(1,1) ;
                w_target + k2*v_target*error(2,1) + k3*v_target*sin(error(3,1))];

%            Vc=[ v_target*cos(error(3,1)) + k1*error(1,1) ;
%                 w_target + k2*v_target*error(2,1) + k3*v_target*sin(error(3,1))];

           VL= Vc(1)-Vc(2)*R;
           VR= Vc(1)+Vc(2)*R;
           
           delta_U    = (VR*0.5+VL*0.5)/2;
           delta_theta= (VR*0.5-VL*0.5)/L;
%             delta_theta= Vc(2)*1;
            disp("----------")
           theta_1=theta_1+delta_theta;
           if(theta_1>2*pi)
               theta_1=theta_1-2*pi;
           end
%            print(theta_1);
           x_1=x_1+delta_U*cos(theta_1);
           y_1=y_1+delta_U*sin(theta_1);

           sprintf('x_1 = %0.3f m',x_1)
           sprintf('y_1 = %0.3f m',y_1)
           plot(x_1,y_1,'o','markersize',0.5,'color','k');
           set(fig_quanang,'xdata',x_1,'ydata',y_1);
           pause(0.02)
end






