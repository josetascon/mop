function [] = plot_camera( R, t )
% 		Jose David Tascón V.
%		Jul 07 2013
s = 0.05;
C = -R'*t;

vx = [0 0 0; s*1 0 0]';
vy = [0 0 0; 0 s*1 0]';
vz = [0 0 0; 0 0 s*1]';
%  warning ("off", "Octave:broadcast");
%  vxn = [vx(:,1) rotationwithquaternion(vx(:,2),Q)] + t;
%  vyn = [vy(:,1) rotationwithquaternion(vy(:,2),Q)] + t;
%  vzn = [vz(:,1) rotationwithquaternion(vz(:,2),Q)] + t;
vxn = bsxfun(@plus,R'*vx,C); %(R*vx + t);
vyn = bsxfun(@plus,R'*vy,C); %(R*vy + t);
vzn = bsxfun(@plus,R'*vz,C); %(R*vz + t);
%  warning ("on", "Octave:broadcast");

%  figure;
plot3( vxn(1,:), vxn(2,:), vxn(3,:), '-r');
hold on;
plot3( vyn(1,:), vyn(2,:), vyn(3,:), '-g');
hold on;
plot3( vzn(1,:), vzn(2,:), vzn(3,:), '-b');
%  xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);