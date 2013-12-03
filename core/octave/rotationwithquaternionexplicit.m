function [vn] = rotationwithquaternionexplicit( v, q )
% Basics on Rotations, quaternion conjugate. Explicit formulation calculated on paper
% vn = q*v*qconj (vn is the new vector rotated)
% 		Jose David Tascón V.
%		Aug 28 2013
[rows cols] = size(v);
assert(rows == 3 & cols == 1, 'Error, vector has to be a vector [3x1]');
[rows cols] = size(q);
assert(rows == 4 & cols == 1, 'Error, quaternion has to be a vector [4x1]');
vn = [0;0;0];
% rotation solve by me (unitary quaternion). Runs better in matlab.
w=q(1);x=q(2);y=q(3);z=q(4);
a=v(1);b=v(2);c=v(3);
t11=w*w;
t22=x*x;
t33=y*y;
t44=z*z;
t12=w*x;
t13=w*y;
t14=w*z;
t23=x*y;
t24=x*z;
t34=y*z;

vn(1) = a*(t11+t22-t33-t44) + 2*( (t13+t24)*c + (t23-t14)*b );
vn(2) = b*(t11-t22+t33-t44) + 2*( (t14+t23)*a + (t34-t12)*c );
vn(3) = c*(t11-t22-t33+t44) + 2*( (t12+t34)*b + (t24-t13)*a );

%  % rotation equations done in ceres (unitary quaternion)
%  t2 =  q(1) * q(2);
%  t3 =  q(1) * q(3);
%  t4 =  q(1) * q(4);
%  t5 = -q(2) * q(2);
%  t6 =  q(2) * q(3);
%  t7 =  q(2) * q(4);
%  t8 = -q(3) * q(3);
%  t9 =  q(3) * q(4);
%  t1 = -q(4) * q(4);
%  vn(1) = 2 * ((t8 + t1) * v(1) + (t6 - t4) * v(2) + (t3 + t7) * v(3)) + v(1);
%  vn(2) = 2 * ((t4 + t6) * v(1) + (t5 + t1) * v(2) + (t9 - t2) * v(3)) + v(2);
%  vn(3) = 2 * ((t7 - t3) * v(1) + (t2 + t9) * v(2) + (t5 + t8) * v(3)) + v(3);