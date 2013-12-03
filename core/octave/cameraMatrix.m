function [P] = cameraMatrix( x, St )
% Camera Matrix,
% 	Jose David Tascón V.
%	Sep 24 2013

[rows_x, cols_x] = size(x);
assert(cols_x == 3, 'Error, Data in x has to be a [nx3] array');
[rows_St, cols_St] = size(St);
assert(cols_St == 4, 'Error, Data in St has to be a [nx4] array');
assert(rows_x == rows_St, 'Error, x & St must have the same length n (number of rows)');

n = rows_x; % Number of points

A = zeros(2*n,12);

%  T1 = normalizeData( x' );
%  T2 = normalizeData( xd2' );

%  x1 = (T1*xd1')';
%  x2 = (T2*xd2')';

v4_zero = zeros(1,4);

for k = 1:n
    A(2*k-1,:) = [ v4_zero -x(k,3)*St(k,:) x(k,2)*St(k,:) ];
    A(2*k,:) = [ x(k,3)*St(k,:) v4_zero -x(k,1)*St(k,:) ];
end;

[U, S, V] = svd(A);
P = [V(1:4,12)';V(5:8,12)';V(9:12,12)'];
P = P/P(3,4);

% Normalization
%  Prcv_k = inv(k)*Prcv;
%  Rrcv = Prcv_k(1:3,1:3);
%  nv = 1/norm(Rrcv);
%  sd = sign(det(Rrcv));
%  trcv = sd*nv*Prcv_k(:,4);
%  RrcvT = sd*nv*Rrcv;