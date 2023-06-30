clc
close all
clear all
%% Khởi tạo thông số
k1=0.0045;k2=0.00589;k3=0.00455;
Ld12=0.2;Psid_12=(280*pi)/180;Ld_13=0.2;Ld_23=0.2;

%% Khởi tạo bản vẽ
figure('name','Chuyển động','color','white','numbertitle','on');
hold on
grid on

xlim([-20 30]); ylim([-20 30]);

%% Position of vehicles
% Xe số 1
x_1=0.5;y_1=0.5;theta_1=(10/180)*pi;
fig_num1 = plot(x_1,y_1,'ro','MarkerSize',5,'markerfacecolor','r');

% Xe số 2
x_2=0;y_2=0.5;theta_2=(70/180)*pi;
fig_num2 = plot(x_2,y_2,'ro','MarkerSize',5,'markerfacecolor','yellow');

% Xe số 3
x_3=0.5;y_3=0;theta_3=(30/180)*pi;
fig_num3 = plot(x_3,y_3,'ro','MarkerSize',5,'markerfacecolor','b');


% Khởi tạo tạo độ điểm target 
x_target=6.95;y_target=8.24;theta_target=(-27/180)*pi;
v_target=0.0;w_target=0.0;
plot(x_target,y_target,'s','MarkerSize',10,'markerfacecolor','blue');

x2_target=20;y2_target=2;theta2_target=(-27/180)*pi;
plot(x2_target,y2_target,'s','MarkerSize',10,'markerfacecolor','blue');

%% Thông số xe 
R =0.03255;
L =0.11;
d =0.0442;

%% Phương trình đường thẳng y=ax+b là điểm nối từ điểm hiện tại đến điểm mục tiêu

anpha_target=anpha_target(x_1,y_1,x_target,y_target);
%% Xoay xe
theta_1 = anpha_target;
theta_2 = anpha_target;
theta_3 = anpha_target;
% while ((abs(x_1)<=abs(x_target))&&(abs(y_1)<=abs(y_target)))
while ( (round(x_1,2)~=round(x_target,2)) && (round(y_1,2)~=round(y_target,2)))
           A  =[cos(theta_1) sin(theta_1) 0;
               -sin(theta_1) cos(theta_1) 0;
               0             0            1];
           B  =[x_target-x_1;
                y_target-y_1;
                theta_target-theta_1];
           error=A*B;

           Vc_1=[ v_target*cos(error(3,1)) + k1*error(1,1) ;
                w_target + k2*v_target*error(2,1) + k3*v_target*sin(error(3,1))];

%            Vc=[ v_target*cos(error(3,1)) + k1*error(1,1) ;
%                 w_target + k2*v_target*error(2,1) + k3*v_target*sin(error(3,1))];
           value_sum=connec_robot(x_1,y_1,theta_1,x_2,y_2,theta_2,x_3,y_3,theta_3,Ld12,Psid_12,Ld_13,Ld_23,Vc_1(1),Vc_1(2));

           VL_1= Vc_1(1)-Vc_1(2)*R;
           VR_1= Vc_1(1)+Vc_1(2)*R;

           Vc_2=[value_sum(1);
                 value_sum(2)];

           VL_2= Vc_2(1)-Vc_2(2)*R;
           VR_2= Vc_2(1)+Vc_2(2)*R;

           Vc_3=[value_sum(3);
                 value_sum(4)];

           VL_3= Vc_3(1)-Vc_3(2)*R;
           VR_3= Vc_3(1)+Vc_3(2)*R;


           
           delta_U1    = (VR_1*0.5+VL_1*0.5)/2;
           delta_theta1= (VR_1*0.5-VL_1*0.5)/L;

           delta_U2    = (VR_2*0.5+VL_2*0.5)/2;
           delta_theta2= (VR_2*0.5-VL_2*0.5)/L;

           delta_U3    = (VR_3*0.5+VL_3*0.5)/2;
           delta_theta3= (VR_3*0.5-VL_3*0.5)/L;

%            delta_U1    = (VR_1*1.5+VL_1*1.5)/2;
%            delta_theta1= (VR_1*1.5-VL_1*1.5)/L;
% 
%            delta_U2    = (VR_2*1.5+VL_2*1.5)/2;
%            delta_theta2= (VR_2*1.5-VL_2*1.5)/L;
% 
%            delta_U3    = (VR_3*1.5+VL_3*1.5)/2;
%            delta_theta3= (VR_3*1.5-VL_3*1.5)/L;

%             delta_theta= Vc(2)*1;
            disp("----------")
           theta_1=theta_1+delta_theta1;
           theta_2=theta_2+delta_theta2;
           theta_3=theta_3+delta_theta3;
%            if(theta_1>2*pi)
%                theta_1=theta_1-2*pi;
%            end
%            print(theta_1);
           x_1=x_1+delta_U1*cos(theta_1);
           y_1=y_1+delta_U1*sin(theta_1);

           x_2=x_2+delta_U2*cos(theta_2);
           y_2=y_2+delta_U2*sin(theta_2);

           x_3=x_3+delta_U3*cos(theta_3);
           y_3=y_3+delta_U3*sin(theta_3);

           sprintf('x_1 = %0.3f m',x_1)
           sprintf('y_1 = %0.3f m',y_1)

           plot(x_1,y_1,'o','markersize',0.5,'color','k');
           set(fig_num1,'xdata',x_1,'ydata',y_1);

           plot(x_2,y_2,'o','markersize',0.5,'color','k');
           set(fig_num2,'xdata',x_2,'ydata',y_2);

           plot(x_3,y_3,'o','markersize',0.5,'color','k');
           set(fig_num3,'xdata',x_3,'ydata',y_3);
           pause(0.002)
end
% % del(anpha_target);
% anpha_target=anpha_target(x_1,y_1,x2_target,y2_target);
% %% Xoay xe
% theta_1 = anpha_target;
% 
% while ( (round(x_1,2)~=round(x2_target,2)) && (round(y_1,2)~=round(y2_target,2)))
%            A  =[cos(theta_1) sin(theta_1) 0;
%                -sin(theta_1) cos(theta_1) 0;
%                0             0            1];
%            B  =[x2_target-x_1;
%                 y2_target-y_1;
%                 theta2_target-theta_1];
%            error=A*B;
% 
%            Vc_1=[ v_target*cos(error(3,1)) + k1*error(1,1) ;
%                 w_target + k2*v_target*error(2,1) + k3*v_target*sin(error(3,1))];
% 
% %            Vc=[ v_target*cos(error(3,1)) + k1*error(1,1) ;
% %                 w_target + k2*v_target*error(2,1) + k3*v_target*sin(error(3,1))];
%            value_sum=connec_robot(x_1,y_1,theta_1,x_2,y_2,theta_2,x_3,y_3,theta_3,Ld12,Psid_12,Ld_13,Ld_23,Vc_1(1),Vc_1(2));
% 
%            VL_1= Vc_1(1)-Vc_1(2)*R;
%            VR_1= Vc_1(1)+Vc_1(2)*R;
% 
%            Vc_2=[value_sum(1);
%                  value_sum(2)];
% 
%            VL_2= Vc_2(1)-Vc_2(2)*R;
%            VR_2= Vc_2(1)+Vc_2(2)*R;
% 
%            Vc_3=[value_sum(3);
%                  value_sum(4)];
% 
%            VL_3= Vc_3(1)-Vc_3(2)*R;
%            VR_3= Vc_3(1)+Vc_3(2)*R;
% 
% 
%            
% %            delta_U1    = (VR_1*0.5+VL_1*0.5)/2;
% %            delta_theta1= (VR_1*0.5-VL_1*0.5)/L;
% % 
% %            delta_U2    = (VR_2*0.5+VL_2*0.5)/2;
% %            delta_theta2= (VR_2*0.5-VL_2*0.5)/L;
% % 
% %            delta_U3    = (VR_3*0.5+VL_3*0.5)/2;
% %            delta_theta3= (VR_3*0.5-VL_3*0.5)/L;
% 
%            delta_U1    = (VR_1*1.5+VL_1*1.5)/2;
%            delta_theta1= (VR_1*1.5-VL_1*1.5)/L;
% 
%            delta_U2    = (VR_2*1.5+VL_2*1.5)/2;
%            delta_theta2= (VR_2*1.5-VL_2*1.5)/L;
% 
%            delta_U3    = (VR_3*1.5+VL_3*1.5)/2;
%            delta_theta3= (VR_3*1.5-VL_3*1.5)/L;
% 
% %             delta_theta= Vc(2)*1;
%             disp("----------")
%            theta_1=theta_1+delta_theta1;
%            theta_2=theta_2+delta_theta2;
%            theta_3=theta_3+delta_theta3;
% %            if(theta_1>2*pi)
% %                theta_1=theta_1-2*pi;
% %            end
% %            print(theta_1);
%            x_1=x_1+delta_U1*cos(theta_1);
%            y_1=y_1+delta_U1*sin(theta_1);
% 
%            x_2=x_2+delta_U2*cos(theta_2);
%            y_2=y_2+delta_U2*sin(theta_2);
% 
%            x_3=x_3+delta_U3*cos(theta_3);
%            y_3=y_3+delta_U3*sin(theta_3);
% 
%            sprintf('x_1 = %0.3f m',x_1)
%            sprintf('y_1 = %0.3f m',y_1)
% 
%            plot(x_1,y_1,'o','markersize',0.5,'color','k');
%            set(fig_num1,'xdata',x_1,'ydata',y_1);
% 
%            plot(x_2,y_2,'o','markersize',0.5,'color','k');
%            set(fig_num2,'xdata',x_2,'ydata',y_2);
% 
%            plot(x_3,y_3,'o','markersize',0.5,'color','k');
%            set(fig_num3,'xdata',x_3,'ydata',y_3);
%            pause(0.0002)
% end
% 
% 
% 
% 
% 
% 
% 
% 
% 




