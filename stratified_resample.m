%function [keep] = stratified_resample(w, resample_randstream)
function [keep, Neff] = stratified_resample(w) % , resample_randstream
%
% INPUT:
%   w - set of N weights [w1, w2, ..]
%
% OUTPUTS:
%   keep - N indices of particles to keep 
%   Neff - number of effective particles (measure of weight variance)
%
% Tim Bailey 2004.


w= w / sum(w); % normalise
% Neff= 1 / sum(w .^ 2); 

len= length(w);
keep= zeros(1,len);
%select = stratified_random(len, resample_randstream); %, resample_randstream
select = stratified_random(len);
w= cumsum(w); 

ctr=1; 
for i=1:len
   while ctr<=len & select(ctr)<w(i)
       keep(ctr)= i;
       ctr=ctr+1; 
   end
end
