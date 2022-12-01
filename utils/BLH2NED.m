function [ n,e,d ] = BLH2NED( originBLH,calBLH)
%Lat and Lon are in deg
%Transform the global BLH coordinates to the local station coordinates �˴���ʾ��ϸ˵��

originB = originBLH(1);
originL = originBLH(2);
originH = originBLH(3);

lenth = size(calBLH,1);
n = zeros(lenth,1);
e = zeros(lenth,1);
d = zeros(lenth,1);
for i=1:lenth
    B2 = calBLH(i,1);
    L2 = calBLH(i,2);
    H2 = calBLH(i,3);

    C_EARTH = 6378137.0;
    E2      = 0.0066943799901413156;


    if(abs(originB)>pi)
        originB = DEG2RAD(originB);
    end
    if(abs(originL)>pi)
        originL = DEG2RAD(originL);
    end
    if(abs(B2)>pi)
        B2 = DEG2RAD(B2);
    end
    if(abs(L2)>pi)
        L2 = DEG2RAD(L2);
    end
    deltaB = (B2 - originB);
    deltaL = (L2 - originL);
    deltaH = H2 - originH;


    Rm = C_EARTH * (1 - E2)/power(1 - E2 * sin(originB) * sin(originB) , 1.5);
    Rn = C_EARTH / sqrt(1 - E2 * sin(originB) * sin(originB));

    n(i,1) = (Rm + originH) * deltaB;
    e(i,1) = (Rn + originH) * cos(originB) * deltaL;
    d(i,1) = -deltaH;
end
end

