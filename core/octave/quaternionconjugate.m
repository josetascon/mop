function [qc] = quaternionconjugate( q )
% Basics on Rotations, quaternion conjugate.
% qc = conjugate of q, qc= [w, -x, -y, -z]
% 		Jose David Tascón V.
%		Jun 5 2013
[rows cols] = size(q);
assert(rows == 4 & cols == 1, 'Error, quaternion has to be a vector [4x1]');
qc = q;
qc(2:end) = -1*q(2:end);