function [] = plot_camera_set( R, t )
% 	Jose David Tascón V.
%	Jul 07 2013


%  szR = size(Q);
szR = size(R);

for k = 1:szR(3)
    plot_camera( R(:,:,k), t(:,:,k) );
end;

% plot structure in BA
%  plot3(structure_RCV(:,1), structure_RCV(:,2), structure_RCV(:,3), '*k')