function [Rot] = quaternion2rotation( q )
% Basics on Rotations, quaternion to rotation matrix.
% 		Jose David Tascón V.
%		May 16 2013
[rows cols] = size(q);
assert(rows == 4 & cols == 1, 'Error, quaternion has to be a vector [4x1]');
w = q(1); x = q(2); y = q(3); z = q(4);

Rot = [ [1-2*(y^2+z^2), 2*(x*y-z*w), 2*(x*z+y*w)];
[2*(x*y+z*w), 1-2*(x^2+z^2), 2*(y*z-x*w)];
[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x^2+y^2)]];
