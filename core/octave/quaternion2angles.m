function [ax ay az] = quaternion2angles( q )
% Basics on Rotations, quaternion to rotation matrix.
% 		Jose David Tascón V.
%		July 16 2013
[rows cols] = size(q);
assert(rows == 4 & cols == 1, 'Error, quaternion has to be a vector [4x1]');
%  q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
%  %% NOT WORKING PROPERLY
%  ax = atan2( 2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2) );
%  ay = asin( 2*(q0*q2 - q3*q1) );
%  az = atan2( 2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2) );
%  
%  % rad to degress convertion
%  ax = ax*180/pi;
%  ay = ay*180/pi;
%  az = az*180/pi;


r=quaternion2rotation(q);
[ax,ay,az]= rotation2angles(r);