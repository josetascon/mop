function [x_hom] = normalizeHomogeneous( x )
% Homogeneous Normalization
% 		Jose David Tascón V.
%		Jul 26 2013
[rowx colx] = size(x);

for k = 1:rowx
    x(k,:) = x(k,:) / norm(x(k,colx));
%      x(k,:) = x(k,:) / x(k,colx); %%%% divided by norm to don't change the sign
end;
x_hom = x;