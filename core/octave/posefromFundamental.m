function [Rot1 Rot2 t1 t2] = posefromFundamental( Fundamental, Kalibration )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		May 30 2013

[rows cols] = size(Kalibration);
assert(rows == 3 & cols == 3, 'Error, Calibration Matrix has to be a [3x3] array');
[rows cols] = size(Fundamental);
assert(rows == 3 & cols == 3, 'Error, Fundamental Matrix has to be a [3x3] array');

W = [[0.0, 1.0, 0.0]; [-1.0, 0.0, 0.0]; [0.0, 0.0, 1.0]];
WT = [[0.0, -1.0, 0.0]; [1.0, 0.0, 0.0]; [0.0, 0.0, 1.0]];

e = Kalibration'*Fundamental*Kalibration;
e = e/e(3,3);
%e=-e
[U S V] = svd(e);
Rot1 = U*W*V';

condt = det(Rot1) + 1.0;
if (condt < 0.001 && condt > -0.001)
    e=-e;
    [U S V] = svd(e);
    Rot1 = U*W*V';
end;

Rot2 = U*WT*V';
t1 = U(:,3);
t2 = -1.0*U(:,3);
