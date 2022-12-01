function RAD2DEG = RAD2DEG( rad )
%UNTITLED2 �˴���ʾ�йش˺����ժ�?
%   �˴���ʾ��ϸ˵��
pi = 3.1415926535897932384626433832795;

    if rad > pi
        rad = rad -2 *pi;
    elseif rad < -pi
        rad = rad + 2 * pi;
    end
RAD2DEG =  180.0*rad / pi;

end

