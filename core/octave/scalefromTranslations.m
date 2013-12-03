function [tr_out] = scalefromTranslations( Rot_RCV, t12_RCV, t13_RCV, tr_RCV )
% scale from 1-2 1-3 match
% 		Jose David Tascón V.
%		Aug 09 2013

szc = size(Rot_RCV);
num_cameras = szc(3);

%% Calculate scale of translation
for cam = 1:(num_cameras-2)
    R23 = Rot_RCV(:,:,cam+2)*(Rot_RCV(:,:,cam+1)');
    
    %Minimize the ratio ||a2vn - lambda*a2vd||
    a2vn = -cross(t13_RCV(:,:,cam),R23*t12_RCV(:,:,cam));
    a2vd = cross(t13_RCV(:,:,cam),t12_RCV(:,:,cam+1));
    a2 = 1.0; %initial value
    a2 = a2 + pinv(a2vd)*(a2vn - a2*a2vd); % Normal equations
    %Minimize the ratio ||a1vn - lambda*a1vd||
    a1vn = R23*t12_RCV(:,:,cam) + a2*t12_RCV(:,:,cam+1);
    a1vd = t13_RCV(:,:,cam);
    a1 = 1.0; %initial value
    a1 =  a1 + pinv(a1vd)*(a1vn - a1*a1vd); % Normal equations

    % Update
    t13_RCV(:,:,cam) = a1*t13_RCV(:,:,cam);
    t12_RCV(:,:,cam+1) = a2*t12_RCV(:,:,cam+1);
    tr_RCV(:,:,cam+2) = R23*tr_RCV(:,:,cam+1) + t12_RCV(:,:,cam+1);
end;

tr_out = tr_RCV;