function [q] = rotation2quaternion( Rot )
% Basics on Rotations
% 		Jose David Tascón V.
%		May 16 2013
[rows cols] = size(Rot);
assert(rows == 3 & cols == 3, 'Error, Rotation Matrix has to be a [3x3] array');
r11 = Rot(1,1); r12 = Rot(1,2); r13 = Rot(1,3);
r21 = Rot(2,1); r22 = Rot(2,2); r23 = Rot(2,3);
r31 = Rot(3,1); r32 = Rot(3,2); r33 = Rot(3,3);

q0 = ( r11 + r22 + r33 + 1.0) / 4.0;
q1 = ( r11 - r22 - r33 + 1.0) / 4.0;
q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
q3 = (-r11 - r22 + r33 + 1.0) / 4.0;
if(q0 < 0.0) q0 = 0.0; end;
if(q1 < 0.0) q1 = 0.0; end;
if(q2 < 0.0) q2 = 0.0; end;
if(q3 < 0.0) q3 = 0.0; end;
q0 = sqrt(q0);
q1 = sqrt(q1);
q2 = sqrt(q2);
q3 = sqrt(q3);
if(q0 >= q1 && q0 >= q2 && q0 >= q3)
    q0 = q0 * 1.0;
    q1 = q1 * sign(r32 - r23);
    q2 = q2 * sign(r13 - r31);
    q3 = q3 * sign(r21 - r12);
elseif(q1 >= q0 && q1 >= q2 && q1 >= q3)
    q0 = q0 * sign(r32 - r23);
    q1 = q1 * 1.0;
    q2 = q2 * sign(r21 + r12);
    q3 = q3 * sign(r13 + r31);
elseif(q2 >= q0 && q2 >= q1 && q2 >= q3)
    q0 = q0 * sign(r13 - r31);
    q1 = q1 * sign(r21 + r12);
    q2 = q2 * 1.0;
    q3 = q3 * sign(r32 + r23);
elseif(q3 >= q0 && q3 >= q1 && q3 >= q2)
    q0 = q0 * sign(r21 - r12);
    q1 = q1 * sign(r31 + r13);
    q2 = q2 * sign(r32 + r23);
    q3 = q3 * 1.0;
else
    fprintf('coding error\n');
end;
qout = [q0;q1;q2;q3];
q = quaternionnorm(qout);