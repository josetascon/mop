function [T] = crossMatrix( x )
% tx vector = [t]x matrix
% 		Jose David Tascón V.
%		Jul 26 2013

[rows_x1 cols_x1] = size(x);
assert(rows_x1 == 3 && cols_x1 == 1, 'Error, Data in x has to be a vector [3x1]');

T = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];