% Based on "Finding optimal rotation and translation between corresponding 3D points" from Nghia Ho
% 		Jose David Tascón V.
%		Jun 14 2013

%%%%%%%%% AÑADIR VISUALIZACION

% Generate random n points
n= 25;
Data1 = 2*rand(n,3) - 1; % Row vector of n 3D points
fprintf('Generated random data: \n');
Data1


% Generate rotation and translation
	% angles x,y and z are between 90 and -90 degrees
anglex = 180*rand(1)-90;
angley = 180*rand(1)-90;
anglez = 180*rand(1)-90;
[Rot] = rotation( anglex, angley , anglez );
tr = rand(3,1);

fprintf('Rotations angles are: \n');
fprintf('anglex = %f angley = %f anglez = %f \n', anglez, angley, anglez);
fprintf('Rotation matrix is: \n');
Rot
fprintf('Translation vector is: \n');
tr
fprintf('Program paused. Press enter to continue.\n\n');
pause;

% Safely disable broadcasting warning (temporary)******||
warning ("off", "Octave:broadcast");

% Apply rotation and translation
Data2 = (Rot*Data1' + tr)';
fprintf('Resultant Data that was rotated and translated: \n');
Data2
fprintf('Program paused. Press enter to continue.\n\n');
pause;

% Calculus of Centroids
centroidA = mean(Data1);
centroidB = mean(Data2);

% Matrix H as the covariance matrix
H = (Data1-centroidA)'*(Data2-centroidB);

% Enable broadcasting warning (temporary) *************||
warning ("on", "Octave:broadcast");

% Singular Value Descomposition
[U S V] = svd(H);
% Recovered Rotation Matrix
Rot_f = V*U';
% Check determinant of Rotation
assert( det(Rot_f) - 1 < 1e-8, 'Error, Rotation matrix determinant different of 1');
% Accumulative error in rotation
error_rot = sum(sum(Rot-Rot_f));

fprintf('Recover Rotation matrix is: \n');
Rot_f
fprintf('Accumalive error in Rotation matrix is: %f \n', error_rot);

% Recover translation
tr_f = -Rot_f*centroidA' + centroidB';
% Accumulative error in rotation
error_tr = sum(tr-tr_f);

fprintf('Recover translation vector is: \n');
tr_f
fprintf('Accumalive error in translation vector is: %f \n', error_tr);