function [qout] = quaternionproduct( q1, q2 )
% Basics on Rotations, product of two quaternions, nonconmutative.
% Represents q1*q2.  !!! NO equal to q2*q1
% 		Jose David Tascón V.
%		May 16 2013
[rows1 cols1] = size(q1);
[rows2 cols2] = size(q2);
assert(rows1 == 4 & cols1 == 1, 'Error, quaternion q1 has to be a vector [4x1]');
assert(rows2 == 4 & cols2 == 1, 'Error, quaternion q2 has to be a vector [4x1]');

w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

qout = zeros(rows1,cols1);

qout(1) = w1*w2 - x1*x2 - y1*y2 - z1*z2;
qout(2) = w1*x2 + x1*w2 + y1*z2 - z1*y2;
qout(3) = w1*y2 - x1*z2 + y1*w2 + z1*x2;
qout(4) = w1*z2 + x1*y2 - y1*x2 + z1*w2;