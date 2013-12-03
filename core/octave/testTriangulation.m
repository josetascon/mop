% Test triangulation with synthetic data
% 		Jose David Tascón V.
%		Jun 24 2013

% Generate random n points
n= 25;
X = rand(n,3); % Row vector of n 3D points
fprintf('Generated random data: \n');
X;
X_hom = [X, ones(n,1)]

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
P2 = k*[Rot, tr]
% Fundamental Matrix
fprintf('Fundamental matrix is: \n');
tx = [0 -tr(3) tr(2); tr(3) 0 -tr(1); -tr(2) tr(1) 0];
F = inv(k')*tx*Rot*inv(k);
F = F/F(3,3)
%rank(F)

fprintf('Program paused. Press enter to continue.\n\n');
pause;

x1 = P1*X_hom';
x1 = x1';
% adding some gaussian noise
noise1=0.2*randn(n,2);
x1 = x1+[noise1 zeros(n,1)];
fprintf('Generated x1 data (x1 = P1*X) is: \n');
% Normalize under homogeneous last coordinate
for k = 1:n
    x1(k,:) /= x1(k,3);
end;
%  x1
%  fprintf('Program paused. Press enter to continue.\n\n');
%  pause;

x2 = P2*X_hom';
x2 = x2';
% adding some gaussian noise
noise2=0.2*randn(n,2);
x2 = x2+[noise2 zeros(n,1)];
fprintf('Generated x2 data (x2 = P2*X) is: \n');
% Normalize under homogeneous last coordinate
for k = 1:n
    x2(k,:) /= x2(k,3);
end;
%  x2
%  fprintf('Program paused. Press enter to continue.\n\n');
%  pause;

% Linear Triangulation Algorithm
pts3 = linearTriangulation( P1, P2, x1, x2);
fprintf('Recover X from Triangulation: \n');
pts3
% Final error estimation
error = (X_hom - pts3).^2;
error = sum(error(:));
fprintf('Accumalive error in triangulation is: %f \n', error);
fprintf('Program paused. Press enter to continue.\n\n');
pause;

% Linear Normalized Triangulation Algorithm
pts3norm = linearTriangulation_normalized( P1, P2, x1, x2);
fprintf('Recover X from Normalized Triangulation: \n');
pts3norm
% Final error estimation
error2 = (X_hom - pts3norm).^2;
error2 = sum(error2(:));
fprintf('Accumalive error in normalized triangulation is: %f \n', error2);
fprintf('Program paused. Press enter to continue.\n\n');
pause;

% Optimal Linear Normalized Triangulation Algorithm
fprintf('Recover X from Optimal Triangulation: \n');
[pts3_opt x1_new x2_new] = optimalTriangulation( P1, P2, x1, x2, F); %% remove ; to show new x1 and x2 as optimal.
%  pts3_opt 
error4 = (X_hom - pts3_opt).^2;
error4 = sum(error4(:));
fprintf('Accumalive error in optimal triangulation is: %f \n', error4);
fprintf('Program paused. Press enter to continue.\n\n');
pause;

%  norm(error2 - error)
%  norm(error2 - error3)

% Normalized Version is better than (just) linear
% Normalized Version = Optimal (But optimal modifies x1 and x2 to fulfill better the epipolar constraint x2'*F*x1=0 *** without normalizing x1 and x2)


%% TESTING A THREE VIEW TRIANGULATION
fprintf('Program paused. Press enter to continue.\n\n');
pause;
fprintf('TESTING A THREE VIEW TRIANGULATION.\n');
anglex = 180*randn(1)-90;
angley = 180*rand(1)-90;
anglez = 180*rand(1)-90;
[Rot] = rotation( anglex, angley , anglez );
tr = rand(3,1);
R1 = Rot; t1 = tr;
anglex = 180*rand(1)-90;
angley = 180*rand(1)-90;
anglez = 180*rand(1)-90;
[Rot] = rotation( anglex, angley , anglez );
tr = rand(3,1);
R2 = Rot; t2 = tr;

k = [[520.28075824744883 0.0 240.59418282611989]; [ 0.0 517.35099060486289 240.23529733631955]; [0.0 0.0 1.0]];
fprintf('Camera matrix P1 is: \n');
P1 = [k , zeros(3,1)]
fprintf('Camera matrix P2 is: \n');
P2 = k*[R1, t1]
fprintf('Camera matrix P3 is: \n');
P3 = k*[R2, t2]

x1 = P1*X_hom';
x1 = x1';
% Adding gaussian noise
noise1=2.0*randn(n,2);
x1 = x1 + [noise1 zeros(n,1)];

for k = 1:n
    x1(k,:) /= x1(k,3);
end;
x2 = P2*X_hom';
x2 = x2';
% Adding gaussian noise
noise2=2.0*randn(n,2);
x2 = x2 + [noise2 zeros(n,1)];

for k = 1:n
    x2(k,:) /= x2(k,3);
end;
x3 = P3*X_hom';
x3 = x3';
% Adding gaussian noise
noise3=2.0*randn(n,2);
x3 = x3 + [noise3 zeros(n,1)];

for k = 1:n
    x3(k,:) /= x3(k,3);
end;

%  % Normalization matrices
%  T1 = normalizeData( x1' );
%  T2 = normalizeData( x2' );
%  T3 = normalizeData( x3' );
%  xnorm1 = (T1*x1')';
%  xnorm2 = (T2*x2')';
%  xnorm3 = (T3*x3')';
%  %Triangulation
%  P1 = T1*P1; P2 = T2*P2; P3 = T3*P3;
%  pts3 = zeros(n,4);
%  
%  for k = 1:n
%      A = zeros(6,4);
%      A(1,:) = xnorm1(k,1)*P1(3,:) - P1(1,:);
%      A(2,:) = xnorm1(k,2)*P1(3,:) - P1(2,:);
%      A(3,:) = xnorm2(k,1)*P2(3,:) - P2(1,:);
%      A(4,:) = xnorm2(k,2)*P2(3,:) - P2(2,:);
%      A(5,:) = xnorm3(k,1)*P3(3,:) - P3(1,:);
%      A(6,:) = xnorm3(k,2)*P3(3,:) - P3(2,:);
%      
%      [U S V] = svd(A);
%      pts3(k,:) = V(:,4)/V(4,4);
%  end;
%  pts3

PM = zeros(3,4,3);
PM(:,:,1)=P1;
PM(:,:,2)=P2;
PM(:,:,3)=P3;

pts3MV = zeros(n,4);
for k = 1:n
    x = zeros(3,3);
    x(1,:) = x1(k,:);
    x(2,:) = x2(k,:);
    x(3,:) = x3(k,:);
    pts3MV(k,:) = linearTriangulationMV(PM,x);
end;
mv_error = (pts3MV - X_hom).^2;
mv_error = sum(mv_error(:));
fprintf('Accumalive error in Multiple View triangulation is: %f \n', mv_error);

% Normalization matrices
T1 = normalizeData( x1' );
T2 = normalizeData( x2' );
T3 = normalizeData( x3' );
x1 = (T1*x1')';
x2 = (T2*x2')';
x3 = (T3*x3')';
%Triangulation
P1 = T1*P1; P2 = T2*P2; P3 = T3*P3;

PM = zeros(3,4,3);
PM(:,:,1)=P1;
PM(:,:,2)=P2;
PM(:,:,3)=P3;

pts3MVn = zeros(n,4);
for k = 1:n
    x = zeros(3,3);
    x(1,:) = x1(k,:);
    x(2,:) = x2(k,:);
    x(3,:) = x3(k,:);
    pts3MVn(k,:) = linearTriangulationMV(PM,x);
end;
mv_error = (pts3MVn - X_hom).^2;
mv_error = sum(mv_error(:));
fprintf('Accumalive error in Multiple View Normalized triangulation is: %f \n', mv_error);