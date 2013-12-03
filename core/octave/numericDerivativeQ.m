function [d] = numericDerivativeQ( X, q )

%% Numerical derivative with respect to x
tic;
epsilon = 1e-4;
pe = q;
pe(1) = q(1)+epsilon;
d1 = costQuaternion(X,pe);
pe = q;
pe(1) = q(1)-epsilon;
dwn = (d1 - costQuaternion(X,pe))./(2*epsilon);
toc;

%% Analytical derivative with respect to w
tic;
w=q(1);x=q(2);y=q(3);z=q(4);
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

t98 = t11 + t22 + t33 + t44;
t99 = t98*t98;
dw = zeros(3,1);
dw(1) = 2*(X(1)*w*(t98 - t11 - t22 + t33 + t44) + X(2)*(-t23*2*w - z*(t98 - 2*t11)) + X(3)*(y*(t98 - 2*t11) - t24*2*w))/t99;
dw(2) = 2*(X(2)*w*(t98 - t11 + t22 - t33 + t44) + X(3)*(-t34*2*w - x*(t98 - 2*t11)) + X(1)*(z*(t98 - 2*t11) - t23*2*w))/t99;
dw(3) = 2*(X(3)*w*(t98 - t11 + t22 + t33 - t44) + X(1)*(-t24*2*w - y*(t98 - 2*t11)) + X(2)*(x*(t98 - 2*t11) - t34*2*w))/t99;
toc;

%% Numerical derivative with respect to x
tic;
epsilon = 1e-4;
pe = q;
pe(2) = q(2)+epsilon;
d1 = costQuaternion(X,pe);
pe = q;
pe(2) = q(2)-epsilon;
dxn = (d1 - costQuaternion(X,pe))./(2*epsilon);
toc;

%% Analytical derivative with respect to x
tic;
dx = zeros(3,1);
dx(1) = 2*(X(1)*x*(-t11 + t98 - t22 + t33 + t44) + X(2)*(y*(t98 - 2*t22) + t14*2*x) + X(3)*(-t13*2*x  + z*(t98 - 2*t22)))/t99;
dx(2) = 2*(X(2)*x*(-t11 - t98 + t22 - t33 + t44) + X(3)*(-t34*2*x - w*(t98 - 2*t22)) + X(1)*(-t14*2*x + y*(t98 - 2*t22)))/t99;
dx(3) = 2*(X(3)*x*(-t11 - t98 + t22 + t33 - t44) + X(1)*(z*(t98 - 2*t22) + t13*2*x) + X(2)*(w*(t98 - 2*t22) - t34*2*x))/t99;
toc;

%% Numerical derivative with respect to y
tic;
epsilon = 1e-4;
pe = q;
pe(3) = q(3)+epsilon;
d1 = costQuaternion(X,pe);
pe = q;
pe(3) = q(3)-epsilon;
dyn = (d1 - costQuaternion(X,pe))./(2*epsilon);
toc;

%% Analytical derivative with respect to y
tic;
dy = zeros(3,1);
dy(1) = 2*(X(1)*y*(-t11 - t22 - t98 + t33 + t44) + X(2)*(x*(t98 - 2*t33) + t14*2*y) + X(3)*(w*(t98 - 2*t33) - t24*2*y))/t99;
dy(2) = 2*(X(2)*y*(-t11 + t22 + t98 - t33 + t44) + X(3)*(z*(t98 - 2*t33) + t12*2*y) + X(1)*(-t14*2*y + x*(t98 - 2*t33)))/t99;
dy(3) = 2*(X(3)*y*(-t11 + t22 - t98 + t33 - t44) + X(1)*(-t24*2*y - w*(t98 - 2*t33)) + X(2)*(-t12*2*y + z*(t98 - 2*t33)))/t99;
toc;

%% Numerical derivative with respect to z
tic;
epsilon = 1e-4;
pe = q;
pe(4) = q(4)+epsilon;
d1 = costQuaternion(X,pe);
pe = q;
pe(4) = q(4)-epsilon;
dzn = (d1 - costQuaternion(X,pe))./(2*epsilon);
toc;

%% Analytical derivative with respect to z
tic;
dz = zeros(3,1);
dz(1) = 2*(X(1)*z*(-t11 - t22 + t33 - t98 + t44) + X(2)*(-t23*2*z - w*(t98 - 2*t44)) + X(3)*(-t13*2*z + x*(t98 - 2*t44)))/t99;
dz(2) = 2*(X(2)*z*(-t11 + t22 - t33 - t98 + t44) + X(3)*(y*(t98 - 2*t44) + t12*2*z) + X(1)*(w*(t98 - 2*t44) - t23*2*z))/t99;
dz(3) = 2*(X(3)*z*(-t11 + t22 + t33 + t98 - t44) + X(1)*(x*(t98 - 2*t44) + t13*2*z) + X(2)*(-t12*2*z + y*(t98 - 2*t44)))/t99;
toc;


%  xp(1) = X(1)*(t11+t22-t33-t44) + 2*( (t13+t24)*X(3) + (t23-t14)*X(2) );
%  xp(2) = X(2)*(t11-t22+t33-t44) + 2*( (t14+t23)*X(1) + (t34-t12)*X(3) );
%  xp(3) = X(3)*(t11-t22-t33+t44) + 2*( (t12+t34)*X(2) + (t24-t13)*X(1) );

