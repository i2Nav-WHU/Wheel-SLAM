%function s= multivariate_gauss(x,P,n,randstream)
function s= multivariate_gauss(x,P,n)%, randstream
%
% INPUTS: 
%   (x, P) mean vector and covariance matrix
%   obtain n samples
% OUTPUT:
%   sample set
%
% Random sample from multivariate Gaussian distribution.
% Adapted from MVNORMRND (c) 1998, Harvard University.
%
% Tim Bailey 2004.

len = length(x);
S   = chol(P)';
%X   = randn(randstream, len,n); 
X   = randn( len,n);
s   = S*X + x*ones(1,n);% Generate values from a normal distribution with mean x
%       and standard deviation S.
