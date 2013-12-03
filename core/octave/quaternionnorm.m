function [qout] = quaternionnorm( q1 )
% Basics on Rotations, quaternion normalization
% 		Jose David Tascón V.
%		May 16 2013
[rows cols] = size(q1);
assert(rows == 4 & cols == 1, 'Error, quaternion has to be a vector [4x1]');

qout = q1/norm(q1);
