function [X] = linearTriangulationMV( P, x )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		Jul 29 2013
%	P = Projection Camera Matrix
%	x = Points seen from P


[cam cols_x] = size(x);
assert(cols_x == 3, 'Error, Data in x has to be a [#camx3] array');

[rows_P cols_P camP] = size(P);
assert(rows_P == 3 & cols_P == 4, 'Error, Camera Matrix P has to be a [3x4] matrix');
assert(camP == cam, 'Error, Diffent number of cameras in P and x');

n = camP; % Number of cameras

X = zeros(1,4);
A = zeros(2*n,4);

for k = 1:n
    A(2*k-1,:) = x(k,1)*P(3,:,k) - P(1,:,k);
    A(2*k,:) = x(k,2)*P(3,:,k) - P(2,:,k);
end;
[U S V] = svd(A);

X = V(:,4)/V(4,4);