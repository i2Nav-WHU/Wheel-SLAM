%
% Read binary ('double') file in MatLab
% 
% function lhs = read_bin(fname, n_col)
%
% by Eun-Hwan Shin, Sept. 9, 2002 
%
function lhs = read_bin(fname, n_col)

if nargin ~= 2
    disp('Needs more parameter(s)!')
    lhs = 0;
    return;
end
    
f_id = fopen(fname, 'rb');
if f_id == -1
    disp('Cannot open the file!');
    lhs = 0;
    return;
end

[lhs, eof] = fread(f_id, [n_col,inf], 'double');
fclose(f_id);
clear f_id;
