function [Rot Rx Ry Rz] = rotation( anglex, angley, anglez )
% Basics on Rotations
% 		Jose David Tascón V.
%		May 16 2013

Rx = [1 0 0; 0 cosd(anglex) -sind(anglex); 0 sind(anglex) cosd(anglex)];
Ry = [cosd(angley) 0 sind(angley); 0 1 0; -sind(angley) 0 cosd(angley)];
Rz = [cosd(anglez) -sind(anglez) 0; sind(anglez) cosd(anglez) 0; 0 0 1];

Rot = Rx*Ry*Rz;