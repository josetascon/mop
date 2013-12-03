function [adj_y1,adj_y2] = liu_HartleySturm(y1,y2,arg3,arg4),
%
% [adj_y1,adj_y2] = liu_HartleySturm(y1,y2,arg3[,arg4])
%
% Given the homogeneous coordinates of two image points y1 and y2 and a
% fundamental matrix F, this function computes two new image coordiantes
% yy1 and yy2 such that they satistfy the epipolar constraint
%
% adj_y1' * F * adj_y2 = 0
%
% and the sum of their squared distances to y1 and y2, in the 2D image, is
% minimal.  This optimization step is proposed by Hartley and Sturm and can
% be used for optimal tringulation.
%
% The funtion has two forms:
%
% [adj_y1,adj_y2]=liu_HartleySturm(y1,y2,F);    F is the fundamental matrix
%
% [adj_y1,adj_y2]=liu_HartleySturm(y1,y2,C1,C2);    C1 and C2 are the camera matrices
% from which F is computed.
%
% NOTE: all homogeneous 3D coordinates are of 1-LAST type.
% NOTE: x is not normalized.
% NOTE: requires vgg on path
%
% Klas Nordberg 2009-11-30

if (nargin==3) F1 = arg3; end,
if (nargin==4) F1 = liu_F_from_C(arg3,arg4); end,

% Convert to previous 1-LAST encoding of homogeneous coord
y1=y1([3 1 2]);
y2=y2([3 1 2]);
F1=F1([3 1 2],[3 1 2]);
  
y1 = y1/y1(1);
y2 = y2/y2(1);

% (i)

T1 = [-y1 [0 0;eye(2)]];
T2 = [-y2 [0 0;eye(2)]];

% (ii)

F2 = inv(T1)'*F1*inv(T2);

% (iii)

[U S V] = svd(F2);
e1 = U(:,3); e1 = e1/norm(e1(2:3));
e2 = V(:,3); e2 = e2/norm(e2(2:3));

% (iv)

R1 = [1 0 0;0 e1(2) e1(3);0 -e1(3) e1(2)];
R2 = [1 0 0;0 e2(2) e2(3);0 -e2(3) e2(2)];

% (v)

F3 = R1*F2*R2';

% F3 now has the form
%
% F3 = [d     -f2*d   c;
%       -f1*d f2*f1*d -f2*c;
%       b     -f2*b   a];

% (vi)

f1 = R1*e1; f1=f1(1);
f2 = R2*e2; f2=f2(1);

a = F3(3,3);
b = F3(3,1);
c = F3(1,3);
d = F3(1,1);

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

adj_y1 = inv(T1)*R1'*yy1;
adj_y2 = inv(T2)*R2'*yy2;

% yyy1 and yyy2 now satisfy yyy1'*F1*yyy2=0
% and have shortest total squared Euclidean distance to y1, y2

% Change back to 1-LAST encoding to homogeneous coordinates

adj_y1 = adj_y1([2 3 1]);
adj_y2 = adj_y2([2 3 1]);

return
