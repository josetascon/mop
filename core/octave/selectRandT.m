function [Rot tr] = selectRandT( Rot1, Rot2, t1, t2, K, x1, x2 )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		Jul 26 2013

% Select rotation as constrained version
[ax ay az] = rotation2angles(Rot1);
a1 = [ax;ay;az];
[ax ay az] = rotation2angles(Rot2);
a2 = [ax;ay;az];
if ( norm(a1) < norm(a2) )
Rot = Rot1;
else
Rot = Rot2;
end;

P1 = [K zeros(3,1)];
P2t1 = K*[Rot t1];
P2t2 = K*[Rot t2];
pt3_t1 = linearTriangulation_normalized( P1, P2t1, x1, x2 );
pt3_t2 = linearTriangulation_normalized( P1, P2t2, x1, x2 );

countpos1 = (pt3_t1(:,3) > 0.0);
countpos1 = sum(countpos1);
countpos2 = (pt3_t2(:,3) > 0.0);
countpos2 = sum(countpos2);

if ( countpos1 >= countpos2 )
tr = t1;
else
tr = t2;
end;