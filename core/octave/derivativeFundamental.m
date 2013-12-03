function [d] = derivativeFundamental( x, p)
% Fundamental Matrix, Eigth Point Algorithm, Gold standard HZ_2004
% 		Jose David Tascón V.
%		Jul 29 2013
%		P1 = x = [I | 0]
%		p arrange as p = [P2(:); X(:,1:3)]

%  %numerical derivative
%  epsilon = 1e-4;
%  pe = p;
%  pe(1) = p(1)+epsilon;
%  d1 = costFundamental(x,pe);
%  pe = p;
%  pe(1) = p(1)-epsilon;
%  d1 = (d1 - costFundamental(x,pe))./(2*epsilon);
%  
%  
%  p(1) = 1;
%  p(2:12) = zeros(11,1);
%  P2 = reshape(p(1:12), 3, 4);
%  
%  struct = p(13:end);
%  num_features = length(struct)/3;
%  
%  struct = reshape(struct,num_features,3);
%  struct = ([struct ones(num_features,1)])';
%  
%  
%  f = [(zeros(3,4)*struct)' ; (P2*struct)'];
%  %  f = normalizeHomogeneous(f);
%  
%  d1
%  d = f