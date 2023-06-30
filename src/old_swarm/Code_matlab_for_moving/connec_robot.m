function u=connec_robot(x1,y1,theta1,x2,y2,theta2,x3,y3,theta3,Ld_12,Psid_12,Ld_13,Ld_23,V1,U1)
d=0.0442;
K=[0.00084 0.0007 0.0001 0.0007];
L12_x=x1-x2-d*cos(theta1);
L12_y=y1-y2-d*sin(theta1);
L13_x=x1-x3-d*cos(theta1);
L13_y=y1-y3-d*sin(theta1);
L23_x=x2-x3-d*cos(theta2);
L23_y=y2-y3-d*sin(theta2);

L12=sqrt(L12_x^2+L12_y^2);
L13=sqrt(L13_x^2+L13_y^2);
L23=sqrt(L23_x^2+L23_y^2);

Psi_12=atan(L12_y/L12_x)-theta1+pi;
Psi_13=atan(L13_y/L13_x)-theta1+pi;
Psi_23=atan(L23_y/L23_x)-theta2+pi;

Beta_12=theta1-theta2;
Beta_13=theta1-theta3;
Beta_23=theta2-theta3;

Gamma_12=Beta_12+Psi_12;
Gamma_13=Beta_13+Psi_13;
Gamma_23=Beta_23+Psi_23;

G=[cos(Gamma_12)      d*sin(Gamma_12)      0              0;
  -sin(Gamma_12)/L12  d*cos(Gamma_12)/L12  0              0;
  0                   0                    cos(Gamma_13)  d*sin(Gamma_13);
  -cos(Psi_23)        0                    cos(Gamma_23)  d*sin(Gamma_23)];

F2=[-cos(Psi_12)     0;
    sin(Psi_12)/L12  -1;
    -cos(Psi_13)   0;
    0                0];

p=[K(1)*(Ld_12-L12);
   K(2)*(Psid_12-Psi_12);
   K(3)*(Ld_13-L13);
   K(4)*(Ld_23-L23)];
u1=[V1;U1];
u23=inv(G)*(p-F2*u1);
u=u23;


