% Test triangulation with synthetic data
% 		Jose David Tascón V.
%		Jun 24 2013

% *** Works changing in linearTriangulation /// pts3(k+1,:) = V(:,4)/V(3,4); 

% Generate random n points
n= 25;
x1 = 480*rand(n,2); % Row vector of n 2D points (assuming a 480x480 image)
fprintf('Generated random data: \n');
x1;
x1_hom = [x1, ones(n,1)]

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

% Define a constant Kalibration Matrix
k = [[520.28075824744883 0.0 240.59418282611989]; [ 0.0 517.35099060486289 240.23529733631955]; [0.0 0.0 1.0]];
fprintf('Camera matrix P1 is: \n');
P1 = [k , zeros(3,1)]
fprintf('Camera matrix P2 is: \n');
P2 = [k*Rot, tr]
fprintf('Program paused. Press enter to continue.\n\n');
pause;

X = pinv(P1)*x1_hom'; % pinv(P1) size [4x3], then X size = [4xn]
X = X';
fprintf('The 3D Data is: \n');
X
fprintf('Program paused. Press enter to continue.\n\n');
pause;

x2 = P2*X';
x2 = x2';
fprintf('Generated x2 data (x2 = P2*X) is: \n');
x2 = x2(:,1:2);
x2_hom = [x2, ones(n,1)]
fprintf('Program paused. Press enter to continue.\n\n');
pause;


pts3 = linearTriangulation( P1, P2, x1_hom, x2_hom);
fprintf('Recover X from Triangulation: \n');
pts3
fprintf('Program paused. Press enter to continue.\n\n');
pause;

error = X(:,1:3) - pts3(:,1:3);
error = sum(error(:));
fprintf('Accumalive error in triangulation is: %f \n', error);

pts3 = linearTriangulation_normalized( P1, P2, x1_hom, x2_hom);
fprintf('Recover X from Triangulation: \n');
pts3
fprintf('Program paused. Press enter to continue.\n\n');
pause;