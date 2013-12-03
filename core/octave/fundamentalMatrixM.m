function [F] = fundamentalMatrixM( xd1, xd2 )
% Fundamental Matrix, Eigth Point Algorithm, Gold standard HZ_2004
% 		Jose David Tasc??n V.
%		Jul 24 2013

[rows_x1, cols_x1] = size(xd1);
assert(cols_x1 == 3, 'Error, Data in xd1 has to be a [nx3] array');
[rows_x2, cols_x2] = size(xd2);
assert(cols_x2 == 3, 'Error, Data in xd2 has to be a [nx3] array');
assert(rows_x1 == rows_x2, 'Error, x1 & x2 must have the same length n (number of rows)');

n = rows_x1; % Number of points

A = zeros(n,9);

T1 = normalizeData( xd1' );
T2 = normalizeData( xd2' );

x1 = (T1*xd1')';
x2 = (T2*xd2')';

for k = 1:n
    A(k,:) = [x2(k,1)*x1(k,1) , x2(k,1)*x1(k,2) , x2(k,1) ,...
	    x2(k,2)*x1(k,1) , x2(k,2)*x1(k,2) , x2(k,2) ,...
	    x1(k,1) , x1(k,2) 1];
end;

[U, S, V] = svd(A);
Ftmp = [V(1:3,9)';V(4:6,9)';V(7:9,9)'];
[Uf, Sf, Vf] = svd(Ftmp);
Sf(3,3) = 0;
Ftmp = Uf*Sf*Vf';
Fp = T2'*Ftmp*T1;
Fp = Fp/Fp(3,3); 
F = Fp;

%  [U, S, V] = svd(Fp);
%  % Right null space %  e1 = V(:,3);	% F*e1 = 0 Epipole of image 1
%  % Left null space  %  e2'*F = 0
%  e2 = U(:,3);	
%  
%  P1 = [eye(3) zeros(3,1)];
%  P2 = [crossMatrix(e2)*Fp e2];
%  
%  X = linearTriangulation_normalized( P1, P2, xd1, xd2 );
%  
%  % Observations
%  y = [xd1; xd2]; % Size 2n x 3
%  xin = P1;
%  Xtmp = X(:,1:3);
%  pin = [P2(:); Xtmp(:)];
%  
%  [pout,resnorm] = lsqcurvefit(@costFundamentalM, pin, xin, y);
%  %  fprintf('Residual value %f \n',resnorm);
%  Prcv=reshape(pout(1:12),3,4);
%  F = crossMatrix(Prcv(:,4))*Prcv(:,1:3);
%  F = F/F(3,3);
%  
%  %[x,resnorm] = lsqnonlin(@costFundamentalM,x0);