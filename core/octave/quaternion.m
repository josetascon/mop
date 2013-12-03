function [qout] = quaternion( anglex, angley, anglez )
% Basics on Rotations, quaternion angle rotation
% 		Jose David Tascón V.
%		May 16 2013
qx = [ cosd(anglex/2); sind(anglex/2); 0; 0 ];
qy = [ cosd(angley/2); 0; sind(angley/2); 0 ];
qz = [ cosd(anglez/2); 0; 0; sind(anglez/2) ];

qmiddle = quaternionproduct(qy,qz);
qout = quaternionproduct(qx,qmiddle);
%  qmiddle = [1;0;0;0];
%  qmiddle = quaternionproduct(qz,qmiddle);
%  qmiddle = quaternionproduct(qy,qmiddle);
%  qmiddle = quaternionproduct(qx,qmiddle);
%  qout = qmiddle;

%  x = anglex/2;
%  y = angley/2;
%  z = anglez/2;
%  qout = [cosd(x)*cosd(y)*cosd(z) + sind(x)*sind(y)*sind(z);
%      sind(x)*cosd(y)*cosd(z) - cosd(x)*sind(y)*sind(z);
%      cosd(x)*sind(y)*cosd(z) + sind(x)*cosd(y)*sind(z);
%      cosd(x)*cosd(y)*sind(z) - sind(x)*sind(y)*cosd(z)];