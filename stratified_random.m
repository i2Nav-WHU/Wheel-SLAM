%function s = stratified_random(N, resample_randstream)
function s = stratified_random(N) %, resample_randstream
%
% Generate N uniform-random numbers stratified within interval (0,1).
% The set of samples, s, are in ascending order.
%
% Tim Bailey 2003.

k = 1/N;
di = (k/2):k:(1-k/2); % deterministic intervals
%s = di + rand(resample_randstream, 1,N) * k - (k/2); % dither within interval %resample_randstream,
s = di + rand( 1,N) * k - (k/2);
