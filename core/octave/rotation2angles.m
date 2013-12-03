function [anglex angley anglez] = rotation2angles( Rot )
% Basics on Rotations
% 		Jose David Tascón V.
%		May 30 2013

[rows cols] = size(Rot);
assert(rows == 3 & cols == 3, 'Error, Rotation Matrix has to be a [3x3] array');

anglex = atan2( Rot(2,3), Rot(3,3) );
angley = atan2 ( -Rot(1,3), sqrt( Rot(1,1)^2 + Rot(1,2)^2 ) );

s1 = sin(anglex); c1 = cos(anglex);
anglez = atan2( s1*Rot(3,1) - c1*Rot(2,1) , c1*Rot(2,2) - s1*Rot(3,2) );

% rad to degress convertion
anglex = -anglex*180/pi;
angley = -angley*180/pi;
anglez = -anglez*180/pi;