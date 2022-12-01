% Interpolate azimuth (solved the +/- pi jump problem)
% Azimuth=interp1_Azimuth(t_0,Azimuth_0,t);
%
% By Xiaoji Niu, 2004-10-12

function Azimuth=interp1_Azimuth(t_0,Azimuth_0,t)
N=length(Azimuth_0);
A_threshold=pi;
for i=2:N
    if Azimuth_0(i)-Azimuth_0(i-1)>A_threshold
        Azimuth_0(i:N)=Azimuth_0(i:N)-2*pi;
    elseif Azimuth_0(i)-Azimuth_0(i-1)<-A_threshold
        Azimuth_0(i:N)=Azimuth_0(i:N)+2*pi;
    end
end
Azimuth_1=interp1(t_0,Azimuth_0,t);

Azimuth=Azimuth_1-round(Azimuth_1/(2*pi))*2*pi;

% figure;
% subplot(311), plot(t_0, Azimuth_0);
% subplot(312), plot(t, Azimuth_1);
% subplot(313), plot(t, Azimuth);
