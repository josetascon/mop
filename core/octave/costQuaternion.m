function [xp] = costQuaternion( X, q )

qtemp = quaternionnorm(q);
w=qtemp(1);x=qtemp(2);y=qtemp(3);z=qtemp(4);
a=X(1);b=X(2);c=X(3);
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

xp = zeros(3,1);
xp(1) = X(1)*(t11+t22-t33-t44) + 2*( (t13+t24)*X(3) + (t23-t14)*X(2) );
xp(2) = X(2)*(t11-t22+t33-t44) + 2*( (t14+t23)*X(1) + (t34-t12)*X(3) );
xp(3) = X(3)*(t11-t22-t33+t44) + 2*( (t12+t34)*X(2) + (t24-t13)*X(1) );
