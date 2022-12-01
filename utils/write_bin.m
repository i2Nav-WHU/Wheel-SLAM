%
% Write binary ('double') file in MatLab
% 
% function result = write_bin(fname, lhs)
% Input: fname: file name of the destination file;
%       lhs: matrix to be saved.
% Output: result = 1 if success; 0 if fail.
%
% by Xiaoji Niu, May 28, 2003 
%
function result = write_bin(fname, lhs)

if nargin ~= 2
    disp('Needs more parameter(s)!')
    result = 0;
    return;
end
    
f_id = fopen(fname, 'wb');
if f_id == -1
    disp('Cannot open the file!');
    result = 0;
    return;
end

[N,M]=size(lhs);
COUNT = fwrite(f_id, lhs', 'double');
if COUNT ~= M*N
    result=0;
    return;
end

result=1;

fclose(f_id);
clear f_id;

