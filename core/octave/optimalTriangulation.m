function [pts3 x1_new x2_new] = optimalTriangulation( P1, P2, x1, x2, F )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		Jun 24 2013
%	F = Fundamental Matrix
%	x1 = Points seen from P1
%	x2 = Points seen from P2

[rows_x1 cols_x1] = size(x1);
assert(cols_x1 == 3, 'Error, Data in x1 has to be a [nx3] array');
[rows_x2 cols_x2] = size(x2);
assert(cols_x2 == 3, 'Error, Data in x2 has to be a [nx3] array');
assert(rows_x1 == rows_x2, 'Error, x1 & x2 must have the same length n (number of rows)');
[rows_F cols_F] = size(F);
assert(rows_F == 3 & cols_F == 3, 'Error, Fundamental matrix has to be a [3x3] matrix');
assert(rank(F) == 2, 'Error, Fundamental matrix is not a rank 2 matrix');

x1_set = zeros(size(x1));
x2_set = zeros(size(x2));

n=length(x1);

% Taking new set from optimal triangulation
for k = 1:n
    [adj_x1, adj_x2] = optimalTriangulation_point( F, x1(k,:)', x2(k,:)' );
    adj_x1 = adj_x1/adj_x1(3);
    adj_x2 = adj_x2/adj_x2(3);
    x1_set(k,:) = adj_x1';
    x2_set(k,:) = adj_x2';
end

[pts3] = linearTriangulation_normalized( P1, P2, x1_set, x2_set );

x1_new = x1_set;
x2_new = x2_set;
