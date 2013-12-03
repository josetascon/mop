function [f] = costFundamental( x, p)
% Fundamental Matrix, Eigth Point Algorithm, Gold standard HZ_2004
% 		Jose David Tasc??n V.
%		Jul 29 2013
%		P1 = x
%		p arrange as p = [P2(:); X(:,1:3)]

P2 = reshape(p(1:12), 3, 4);

struct = p(13:end);
num_features = length(struct)/3;

struct = reshape(struct,num_features,3);
struct = ([struct ones(num_features,1)])';

f = [(x*struct)' ; (P2*struct)'];
f = normalizeHomogeneous(f);