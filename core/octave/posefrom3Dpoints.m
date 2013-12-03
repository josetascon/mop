function [Rot tr] = posefrom3Dpoints( Xdata1, Xdata2 )
% Basics on Epipolar Geometry
% 		Jose David Tascón V.
%		Jul 08 2013
%	Xdata1 = Projection Camera Matrix 1
%	Xdata2 = Projection Camera Matrix 2

[rows_x1 cols_x1] = size(Xdata1);
assert(cols_x1 == 3, 'Error, Xdata1 has to be a [nx3] array');
[rows_x2 cols_x2] = size(Xdata2);
assert(cols_x2 == 3, 'Error, Xdata2 has to be a [nx3] array');
assert(rows_x1 == rows_x2, 'Error, Xdata1 & Xdata2 must have the same length n (number of rows)');

% Safely disable broadcasting warning (temporary)******||
warning ("off", "Octave:broadcast");
% Calculus of Centroids
centroidA = mean(Xdata1);
centroidB = mean(Xdata2);
% Matrix H as the covariance matrix
H = (Xdata1-centroidA)'*(Xdata2-centroidB);
% Enable broadcasting warning (temporary) *************||
warning ("on", "Octave:broadcast");
% Singular Value Descomposition
[U S V] = svd(H);
% Recovered Rotation Matrix
Rot = V*U';
% Check determinant of Rotation
assert( det(Rot) - 1 < 1e-8, 'Error, Rotation matrix determinant different of 1');
% Recover translation
tr = -Rot*centroidA' + centroidB';