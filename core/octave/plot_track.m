function [handler] = plot_track( orientation, translation, arg3, offset )
% 	Jose David Tascón V.
%	Dic 01 2013
%	orientation is 3*n x 3 matrix with n Rotation matrices
%	translation is n x 3 matrix with n translations

if nargin < 3
  arg3 = '-';		% arg3 line continuity pattern
  offset = 0;
end;
if nargin < 4
  offset = 0;		% offset variable to substract
end;

num_cams = length(translation);
%  center1 = [0,0,0];
data = [];
color = rand(3,1);
hold on;
for k = 0:num_cams-1
    R = orientation( 3*k+1:3*k+3, :);
    t = translation(k+1,:)';
    center2 = -R'*t;
    data(k+1,:) = center2';
%      data = center2';
%      data = [center1; center2'];
%      plot3( data(:,1), data(:,2), data(:,3), '-', 'Color', color );
%      center1 = center2';
end;
if(offset==1) 
    data = bsxfun(@plus,data, data(1,:));
%      data
end;
handler = plot3( data(:,1), data(:,2), data(:,3), arg3, 'Color', color );
xlabel( 'x' ); ylabel( 'y' ); zlabel( 'z' );
grid on;


fprintf('Euclidean distance First->Last = %f \n', norm(data(1,:) - data(num_cams,:)) );
%  printf('Euclidean distance First and Last', norm( data(:,1) - data(:,num_cams) );