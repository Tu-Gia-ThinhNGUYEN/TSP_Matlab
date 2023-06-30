G_target=[5.8;8.0];
Q_can=[4.4;7.2];
Set_first=[3;6];
anpha=atan((G_target(2)-Set_first(2))/(G_target(1)-Set_first(1)));
Position_first_vehicle=[Set_first(1);Set_first(2);deg2rad(19.5)];
% met
% a=rad2deg(atan((G_target(2)-Q_can(2))/(G_target(1)-Q_can(1))));