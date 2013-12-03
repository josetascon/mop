% Test BA synthetic data
% 		Jose David Tascón V.
%		Jun 24 2013
%		Modified 08 August

clear all; close all; clc;
% Definition of num of features and cameras
num_cameras = 24; %multiples of 6. mainly 12 or 24
fxc = 55;
num_features = fxc*num_cameras;
width = 800;
height = 600;

X = zeros(num_features,3); % Row vector of n 3D points
%  fprintf('Generated random Structure data: \n');
%  X;

%Create rotation, translation and cameras
Rotation = zeros(3,3,num_cameras); 
translation = zeros(3,1,num_cameras);
cam_center = zeros(3,1,num_cameras);
Quaternion = zeros(4,1,num_cameras);
Camera = zeros(3,4,num_cameras);

% Define a constant Kalibration Matrix
K = [[500 0.0 width/2]; [ 0.0 500 height/2]; [0.0 0.0 1.0]];
%  K = [[520.28075824744883 0.0 328.59418282611989]; [ 0.0 517.35099060486289 265.23529733631955]; [0.0 0.0 1.0]];

% First camera
ratio = 1.2;
Rotation(:,:,1) = eye(3);
Quaternion(:,:,1) = [1;0;0;0];
cam_center(:,:,1) = [0;0;1*ratio]; % circle translation
translation(:,:,1) = -Rotation(:,:,1)*cam_center(:,:,1);
Camera(:,:,1) = K*[Rotation(:,:,1) translation(:,:,1)];

%%Generate rotation and translation
%%angles x,y and z are between -7 and 7 degrees
%%warning ("off", "Octave:broadcast");
for cam = 2:num_cameras
    anglex = 0;%14*rand(1)-7;
    angley = -(cam-1)*(360/num_cameras);
    anglez = 0;%14*rand(1)-7;
    Quaternion(:,:,cam) = quaternion( anglex, angley , anglez );
    Rotation(:,:,cam) = rotation(anglex,angley,anglez);
%      cam_center(:,:,cam) = Rotation(:,:,cam)*cam_center(:,:,1);
    cam_center(:,:,cam) = [ratio*(sind(-angley));0;ratio*(cosd(-angley))];
%      cam_center(:,:,cam)
    translation(:,:,cam) = -Rotation(:,:,cam)*cam_center(:,:,cam);

    st = rand(fxc,3);
    st(:,1:2) = 4*st(:,1:2)-2;
%      st(:,3) = 2*st(:,3);
%      signz = cam_center(3,:,cam-1)/norm(cam_center(3,:,cam-1));
    X( (cam-2)*(fxc)+1:(cam-1)*fxc,:) = bsxfun(@plus,(st)*Rotation(:,:,cam-1),7*cam_center(:,:,cam-1)'); %st + (translation(:,:,cam-1)*2)'; 
end;
%% last set
st = rand(fxc,3);
st(:,1:2) = 4*st(:,1:2)-2;
%  st(:,3) = 2*st(:,3);
%  signz = cam_center(3,:,num_cameras)/norm(cam_center(3,:,num_cameras));
X( (num_cameras-1)*(fxc)+1:(num_cameras)*fxc,:) = bsxfun(@plus,st*Rotation(:,:,num_cameras),7*cam_center(:,:,num_cameras)');

%% Testing set
%  %  anglex = 5;%14*rand(1)-7;
%  %  angley = 15;
%  %  anglez = 5;%14*rand(1)-7;
%  %  Rotation(:,:,2) = rotation(anglex, angley, anglez); 
%  %  cam_center(:,:,2) = [1;0;1];
%  %  translation(:,:,2) = -Rotation(:,:,2)*cam_center(:,:,2);
%  %  
%  %  anglex = 5;%14*rand(1)-7;
%  %  angley = 30;
%  %  anglez = 5;%14*rand(1)-7;
%  %  Rotation(:,:,3) = rotation(anglex, angley, anglez); 
%  %  cam_center(:,:,3) = [2;0;1];
%  %  translation(:,:,3) = -Rotation(:,:,3)*cam_center(:,:,3);
%  %  
%  %  st = rand(fxc,3);
%  %  st(:,1:2) = 4*st(:,1:2)-2;
%  %  %  st(:,3) = 2*st(:,3);
%  %  X( (0)*(fxc)+1:(1)*fxc,:) = bsxfun(@plus,(st*Rotation(:,:,1)),cam_center(:,:,1)'+[0,0,2]); %st + (translation(:,:,cam-1)*2)'; 
%  %  
%  %  st = rand(fxc,3);
%  %  st(:,1:2) = 4*st(:,1:2)-2;
%  %  %  st(:,3) = 2*st(:,3);
%  %  X( (1)*(fxc)+1:(2)*fxc,:) = bsxfun(@plus,(st*Rotation(:,:,2)),cam_center(:,:,2)'+[0,0,2]); %st + (translation(:,:,cam-1)*2)'; 
%  %  
%  %  st = rand(fxc,3);
%  %  st(:,1:2) = 4*st(:,1:2)-2;
%  %  %  st(:,3) = 2*st(:,3);
%  %  X( (2)*(fxc)+1:(3)*fxc,:) = bsxfun(@plus,(st*Rotation(:,:,3)),1*cam_center(:,:,3)'+[0,0,2]); %st + (translation(:,:,cam-1)*2)'; 


%  num_cameras = 3 %%%************************************************************************************* remember delete this
%  num_features = fxc*num_cameras
%  X = X(1:num_features,:);

% adding translation movement
for cam = 2:num_cameras
    tr = [0;0;-rand(1)/6];
    translation(:,:,cam) = translation(:,:,cam) + tr;
    Camera(:,:,cam) = K*[Rotation(:,:,cam) translation(:,:,cam)];
end;
%  warning ("on", "Octave:broadcast");

X_hom = [X, ones(num_features,1)];

%  %% Print matrices
%  for cam = 1:num_cameras
%      [anglex angley anglez] = rotation2angles( Rotation(:,:,cam) );
%      fprintf('Camera %i, Rotation angles are:\n',cam);
%      fprintf('anglex = %f angley = %f anglez = %f \n', anglex, angley, anglez);
%      fprintf('Camera %i, Rotation matrix is:\n',cam);
%      Rotation(:,:,cam)
%      fprintf('Camera %i, Translation vector is:\n',cam);
%      translation(:,:,cam)
%  end;
%  
%  fprintf('Program paused. Press enter to continue.\n\n');
%  pause;

%% Generated images form projection of X with model Camera
im_x = zeros(num_features,3,num_cameras);
im_x_h = zeros(num_features,3,num_cameras);

for cam = 1:num_cameras
    im_x(:,:,cam) = (Camera(:,:,cam)*X_hom')';
    im_x_h(:,:,cam) = normalizeHomogeneous( im_x(:,:,cam) );
    im_x_h(:,1:2,cam) = im_x_h(:,1:2,cam) + 0.1*randn(num_features,2); %% Adding the noise
    %%fprintf('Camera %i is:\n',cam);
    %%im_x_h(:,:,cam)
end;

%% PLOT generated data
lim_xyz = ratio*[-10 10];
figure;
%%  title('Syntethic Data Reconstruction')
%%  subplot(1,2,1);
plot_camera_set(Rotation, translation);
plot3(X(:,1), X(:,2), X(:,3), '.k');
xlim(lim_xyz); ylim(lim_xyz); zlim(lim_xyz);
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
title('Syntethic Data Reconstruction (Original)');
grid on;



%% Visibility computation
visibility = zeros(num_cameras,num_features);
coordinates = zeros(num_cameras,num_features, 3);
for cam = 1:num_cameras
    for ft = 1:num_features
        uaxis = (im_x_h(ft,1,cam) > 0.0) && (im_x_h(ft,1,cam) < width);
        vaxis = (im_x_h(ft,2,cam) > 0.0) && (im_x_h(ft,2,cam) < height);
        if (uaxis && vaxis)
	  visibility(cam,ft) = 1;
	  coordinates(cam,ft,:) = im_x_h(ft,:,cam); 
        end;
        
    end;
end;

fprintf('The visibility function is given by [camera x features]\n');
visibility;

%Create RECOVER DATA rotation, translation and cameras
Rot_RCV = zeros(3,3,num_cameras); 
t12_RCV = zeros(3,1,num_cameras-1);
t13_RCV = zeros(3,1,num_cameras-2);
tr_RCV = zeros(3,1,num_cameras);

Camera_RCV = zeros(3,4,num_cameras);

%% Recover motion from subsequent images
% First camera
Rot_RCV(:,:,1) = eye(3);

for cam = 1:(num_cameras-1)
    pt1=[];
    pt2=[];
    count = 1;
    for ft = 1:num_features
        if ((visibility(cam,ft) == 1) && (visibility(cam+1,ft) == 1))
	  pt1(count,:) = coordinates(cam,ft,:);
	  pt2(count,:) = coordinates(cam+1,ft,:);
	  count = count + 1;
        end;
    end;
    if( count < 8 ) return;end;
    F=fundamentalMatrixM( pt1, pt2 );
    [R1 R2 t1 t2]=posefromFundamental( F, K );
    [R t] = selectRandT( R1, R2, t1, t2, K, pt1, pt2);
    
    Rot_RCV(:,:,cam+1) = R*Rot_RCV(:,:,cam);
    t12_RCV(:,:,cam) = t;
    
end;
fprintf('Motion continous 1-2 succesful\n');
%% Recover motion from 1 - 3 images
for cam = 1:(num_cameras-2)
    pt1=[];
    pt2=[];
    count = 1;
    for ft = 1:num_features
        if ((visibility(cam,ft) == 1) && (visibility(cam+2,ft) == 1))
	  pt1(count,:) = coordinates(cam,ft,:);
	  pt2(count,:) = coordinates(cam+2,ft,:);
	  count = count + 1;
        end;
    end;
    if( count < 8 ) return;end;
    F=fundamentalMatrixM( pt1, pt2 );
    [R1 R2 t1 t2]=posefromFundamental( F, K );
    [R t] = selectRandT( R1, R2, t1, t2, K, pt1, pt2);
    t13_RCV(:,:,cam) = t;
end;
fprintf('Motion continous 1-3 succesful\n');

t12_RCV = t12_RCV*norm( translation(:,:,2) - Rotation(:,:,2)*translation(:,:,1) );%% trick to adquiere same translation

tr_RCV(:,:,1) = translation(:,:,1); %[0;0;2]; %%Due to initial coordinates
tr_RCV(:,:,2) = Rotation(:,:,2)*translation(:,:,1) + t12_RCV(:,:,1); %t12_RCV(:,:,1); 

[tr_out] = scalefromTranslations( Rot_RCV, t12_RCV, t13_RCV, tr_RCV );
tr_RCV = tr_out;
%  %% Calculate scale of translation
%  for cam = 1:(num_cameras-2)
%      R23 = Rot_RCV(:,:,cam+2)*(Rot_RCV(:,:,cam+1)');
%      
%      %Minimize the ratio ||a2vn - lambda*a2vd||
%      a2vn = -cross(t13_RCV(:,:,cam),R23*t12_RCV(:,:,cam));
%      a2vd = cross(t13_RCV(:,:,cam),t12_RCV(:,:,cam+1));
%      a2 = 1.0; %initial value
%      a2 = a2 + pinv(a2vd)*(a2vn - a2*a2vd); % Normal equations
%      %Minimize the ratio ||a1vn - lambda*a1vd||
%      a1vn = R23*t12_RCV(:,:,cam) + a2*t12_RCV(:,:,cam+1);
%      a1vd = t13_RCV(:,:,cam);
%      a1 = 1.0; %initial value
%      a1 =  a1 + pinv(a1vd)*(a1vn - a1*a1vd); % Normal equations
%  
%      % Update
%      t13_RCV(:,:,cam) = a1*t13_RCV(:,:,cam);
%      t12_RCV(:,:,cam+1) = a2*t12_RCV(:,:,cam+1);
%      tr_RCV(:,:,cam+2) = R23*tr_RCV(:,:,cam+1) + t12_RCV(:,:,cam+1);
%      
%  end;

for cam = 1:(num_cameras)
    Camera_RCV(:,:,cam) = K*[Rot_RCV(:,:,cam) tr_RCV(:,:,cam)];
end;

%% Print matrices to COMPARE
for cam = 1:num_cameras
    fprintf('**** ORIGINAL ****\n');
    [anglex angley anglez] = rotation2angles( Rotation(:,:,cam) );
    fprintf('Camera %i, Rotation angles are:\n',cam);
    fprintf('anglex = %f angley = %f anglez = %f \n', anglex, angley, anglez);
    fprintf('Camera %i, Rotation matrix is:\n',cam);
    Rotation(:,:,cam)
    fprintf('Camera %i, Translation vector is:\n',cam);
    translation(:,:,cam)
    
    fprintf('**** RECOVER ****\n');
    [anglex angley anglez] = rotation2angles( Rot_RCV(:,:,cam) );
    fprintf('Camera %i, Rotation angles are:\n',cam);
    fprintf('anglex = %f angley = %f anglez = %f \n', anglex, angley, anglez);
    fprintf('Camera %i, Rotation matrix is:\n',cam);
    Rot_RCV(:,:,cam)
    fprintf('Camera %i, Translation vector is:\n',cam);
    tr_RCV(:,:,cam)
end;

%% fprintf('Program paused. Press enter to continue.\n\n');
%%  pause;

structure_RCV = zeros(num_features,4);
%% Find structure
for ft = 1:num_features
    xft_coord = [];
    P = [];
    num_view = 1;
    for cam = 1:(num_cameras)
        if (visibility(cam,ft) == 1)
	  xft_coord(num_view,:) = coordinates(cam,ft,:);
	  P(:,:,num_view) = Camera_RCV(:,:,cam);
	  num_view = num_view + 1;
        end;
    end;
    
    if (num_view > 2)
        structure_RCV(ft,:) = linearTriangulationMV(P,xft_coord);
    end;
end;

%Structure
error_3d = bsxfun( @times,((structure_RCV - X_hom).^2),structure_RCV(:,4) ); %filter by the viewable features
error_3d = sum(error_3d(:));
fprintf('Structure reconstruction squared error: %f\n',error_3d);


%% Plot recover data
figure;
%% subplot(1,2,2);
plot_camera_set(Rot_RCV, tr_RCV);
plot3(structure_RCV(:,1), structure_RCV(:,2), structure_RCV(:,3), '.k');
xlim(lim_xyz); ylim(lim_xyz); zlim(lim_xyz);
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
title('Syntethic Data Reconstruction (Recover)');
grid on;


%%% WORK changing sign in t12_rcv