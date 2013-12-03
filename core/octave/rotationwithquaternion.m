function [vn] = rotationwithquaternion( v, q )
% Basics on Rotations, quaternion conjugate.
% vn = q*v*qconj (vn is the new vector rotated)
% 		Jose David Tascón V.
%		Jun 5 2013
[rows cols] = size(v);
assert(rows == 3 & cols == 1, 'Error, vector has to be a vector [3x1]');
[rows cols] = size(q);
assert(rows == 4 & cols == 1, 'Error, quaternion has to be a vector [4x1]');

vn = quaternionproduct(quaternionproduct(q,[0;v]), quaternionconjugate(q));
vn = vn(2:end);