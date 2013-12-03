function [T] = normalizeData( x )
% Normalization with centroid 0
% 		Jose David Tascón V.
%		Jun 24 2013

% Transform taking x's centroid to the origin
x_mean = mean( x(1,:) );
y_mean = mean( x(2,:) );
Ttrans = [ 1 0 -x_mean; 0 1 -y_mean; 0 0 1 ];

% Calculate appropriate scaling factor
x = Ttrans * x;
lengths = sqrt( sum( x(1:2,:).^2 ));
s = sqrt(2) / mean(lengths);

% Transform scaling x to an average length of sqrt(2)
Tscale = [ s 0 0 ; 0 s 0 ; 0 0 1 ];

% Compose the transforms
T = Tscale * Ttrans;