function [adj_x1, adj_x2] = optimalTriangulation_point( F, x1, x2 )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		Jun 24 2013
%	F = Fundamental Matrix
%	x1 = Points seen from P1
%	x2 = Points seen from P2

[rows_x1 cols_x1] = size(x1);
assert(rows_x1 == 3 & cols_x1 == 1, 'Error, Data in x1 has to be a [3x1] vector');
[rows_x2 cols_x2] = size(x2);
assert(rows_x2 == 3 & cols_x2 == 1, 'Error, Data in x2 has to be a [3x1] vector');
[rows_F cols_F] = size(F);
assert(rows_F == 3 & cols_F == 3, 'Error, Fundamental matrix has to be a [3x3] matrix');
assert(rank(F) == 2, 'Error, Fundamental matrix is not a rank 2 matrix');

x1 = x1/x1(3);
x2 = x2/x2(3);

% (i)
%Transformation matrices
T1 = [1 0 -x1(1); 0 1 -x1(2); 0 0 1];
T2 = [1 0 -x2(1); 0 1 -x2(2); 0 0 1];

% (ii)
F2 = inv(T2')*F*inv(T1);

% (iii)
[U S V] = svd(F2);

% Right null space
e1 = V(:,3);	% F*e1 = 0 Epipole of image 1
% Left null space
e2 = U(:,3);	% e2'*F = 0
% Normalization  of epipoles
e1n = norm(e1(1:2));
e1 = e1/e1n;
e2n = norm(e2(1:2));
e2 = e2/e2n;

% (iv)
% Rotation Matrices
R1 = [e1(1) e1(2) 0; -e1(2) e1(1) 0; 0 0 1];
R2 = [e2(1) e2(2) 0; -e2(2) e2(1) 0; 0 0 1];
%  R1*e1
%  R2*e2

% (v)
F3 = R2*F2*R1';
% Variables from F3

% (vi)
f1 = e1(3); f2 = e2(3);
a = F3(2,2); b = F3(2,3); c = F3(3,2); d = F3(3,3);

% (vii)
k1=b*c-a*d;
g=[...
  a*c*k1*f2^4 ...                        %coefficient for t^6
  (a^2+c^2*f1^2)^2+k1*(b*c+a*d)*f2^4 ... %coefficient for t^5
  4*(a^2+c^2*f1^2)*(a*b+c*d*f1^2)+2*a*c*k1*f2^2+b*d*k1*f2^4 ...
  2*(4*a*b*c*d*f1^2+a^2*(3*b^2+d^2*(f1-f2)*(f1+f2))+c^2*(3*d^2*f1^4+b^2*(f1^2+f2^2))) ...
  -a^2*c*d+a*b*(4*b^2+c^2+4*d^2*f1^2-2*d^2*f2^2)+2*c*d*(2*d^2*f1^4+b^2*(2*f1^2+f2^2)) ...
  b^4-a^2*d^2+d^4*f1^4+b^2*(c^2+2*d^2*f1^2) ... %coefficient for t^1
  b*d*k1];
r = real(roots(g));

% (viii)

s=zeros(length(r)+1,1);
for ix=1:length(r),
  t=r(ix);
  s(ix)=t^2/(1+f2^2*t^2)+(c*t+d)^2/((a*t+b)^2+f1^2*(c*t+d)^2);
end
s(length(r)+1) = [1/f2^2+c^2/(a^2+f1^2*c^2)];

[v ix]=sort(s);

% (ix)

if (ix(1)<length(r)+1),
  tmin = r(ix(1));
  l1 = [c*tmin+d;-f1*(c*tmin+d);a*tmin+b];
  l2 = [-tmin;tmin*f2;1];
else
  l1 = [c;-f1*c;a];
  l2 = [-1;f2;0];
end

yy1 = [l1(2)^2+l1(3)^2;-l1(2)*l1(1);-l1(3)*l1(1)]; %yy1 = yy1/yy1(1);
yy2 = [l2(2)^2+l2(3)^2;-l2(2)*l2(1);-l2(3)*l2(1)]; %yy2 = yy2/yy2(1);

% (x)
adj_x1 = inv(T1)*R1'*yy1;
adj_x2 = inv(T2)*R2'*yy2;

%adj_x1 = adj_x1/adj_x1(3);
%adj_x2 = adj_x2/adj_x2(3);

% Test epipolar constraint
efund1=x2'*F*x1;
efund2=adj_x2'*F*adj_x1;