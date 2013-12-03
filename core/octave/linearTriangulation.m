function [pts3] = linearTriangulation( P1, P2, x1, x2 )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		Jun 24 2013
%	P1 = Projection Camera Matrix 1
%	P1 = Projection Camera Matrix 2
%	x1 = Points seen from P1
%	x2 = Points seen from P2


[rows_x1 cols_x1] = size(x1);
assert(cols_x1 == 3, 'Error, Data in x1 has to be a [nx3] array');
[rows_x2 cols_x2] = size(x2);
assert(cols_x2 == 3, 'Error, Data in x2 has to be a [nx3] array');
assert(rows_x1 == rows_x2, 'Error, x1 & x2 must have the same length n (number of rows)');
[rows_P1 cols_P1] = size(P1);
assert(rows_P1 == 3 & cols_P1 == 4, 'Error, Camera Matrix P1 has to be a [3x4] matrix');
[rows_P2 cols_P2] = size(P2);
assert(rows_P2 == 3 & cols_P2 == 4, 'Error, Camera Matrix P2 has to be a [3x4] matrix');

n = rows_x1; % Number of points

pts3 = zeros(n,4);

for k = 1:n
    A = zeros(4,4);
    A(1,:) = x1(k,1)*P1(3,:) - P1(1,:);
    A(2,:) = x1(k,2)*P1(3,:) - P1(2,:);
    A(3,:) = x2(k,1)*P2(3,:) - P2(1,:);
    A(4,:) = x2(k,2)*P2(3,:) - P2(2,:);
    
    [U S V] = svd(A);
    pts3(k,:) = V(:,4)/V(4,4);
end;