function w = gauss_evaluate(v,S,logflag)
%function w = gauss_evaluate(v,S,logflag)
%
% INPUTS:
%   v - a set of innovation vectors.
%   S - the covariance matrix for the innovations.
%   logflag - <optional> - if 1 computes the log-likelihood, otherwise computes 
%           the likelihood.
%
% OUTPUT:
%   w - set of Gaussian likelihoods or log-likelihoods for each v(:,i).
%
% This implementation uses the Cholesky factor of S to compute the likelihoods 
% and so is more numerically stable than a simple full covariance form. 
% This function is identical to gauss_likelihood().
%
% Tim Bailey 2005.

if nargin == 2, logflag = 0; end

D = size(v,1);
Sc = chol(S)';
nin = Sc\v; % normalised innovation
E = -0.5 * sum(nin.*nin, 1); % Gaussian exponential term 
% Note: writing sum(x.*x, 1) is a fast way to compute sets of inner-products.

if logflag ~= 1
    C = (2*pi)^(D/2) * prod(diag(Sc)); % normalising term (makes Gaussian hyper-volume equal 1)
    w = exp(E) / C; % likelihood
else
    C = 0.5*D*log(2*pi) + sum(log(diag(Sc))); % log of normalising term 
    w = E - C; % log-likelihood
end
