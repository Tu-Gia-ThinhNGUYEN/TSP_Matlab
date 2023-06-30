function Vr=CalcDynamicWindow(x,model)
%Calculate DyamicWindow from model and current state
global dt;
%Window by vehicle model
%model(1): Maximum velocity [m/s],model(2): maximum angular velocity [rad/s],model(3): maximum acceleration [m/ss],model(4): maximum angular acceleration [rad/ss],
%model(5): Veloctity resolution [m/s],model(6): angular velocity resolution [rad/s]]    
Vs=[0 model(1) -model(2) model(2)];

%Window by motion model
%x: initial state of robot [x(m),y(m),yaw(Rad),v(m/s),ƒÖ(rad/s)]
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

%Calculation of the final Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ƒÖmin,ƒÖmax]