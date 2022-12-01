%
% Write txt ('double') file in MatLab
% 
% function result = write_bin(fname, lhs)
% Input: fname: file name of the destination file;
%       lhs: matrix to be saved.
% Output: result = 1 if success; 0 if fail.
%
% by Xiaoji Niu, May 28, 2003 
%
function result = write_txt( fname, lhs )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
format long;
if nargin ~= 2
    disp('Needs more parameter(s)!')
    result = 0;
    return;
end

f_id = fopen(fname, 'w');
if f_id == -1
    disp('Cannot open the file!');
    result = 0;
    return;
end

[N,M]=size(lhs);

if(M == 7)
    COUNT = fprintf(f_id, '%.6f %.10f %.10f %.10f %.10f %.10f %.10f\n', lhs');
elseif(M == 2)
    COUNT = fprintf(f_id, '%.10f %.10f\n', lhs');
elseif(M == 3)
    COUNT = fprintf(f_id, '%.6f %.5f %.5f\n', lhs');
elseif(M == 11)
    COUNT = fprintf(f_id, '%.6f %.6f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n', lhs');
elseif(M == 12)
    COUNT = fprintf(f_id, '%.6f %.6f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n', lhs');
elseif(M == 10)
    COUNT = fprintf(f_id, '%.6f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n', lhs');
elseif(M == 8)
    COUNT = fprintf(f_id, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f \n', lhs');
else
    result = 0;
    COUNT  = 0;
end
if COUNT ~= M*N
    result=0;
    return;
end

result=1;

fclose(f_id);
end

