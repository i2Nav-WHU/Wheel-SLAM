function [atRow, atCol] = getCurrentMapIndex(particle, gridlen)

    tmpN      = particle.xv(1);%current position of the vehicle
    tmpE      = particle.xv(2);
    originrow = particle.originIdx(1); % row idx of the original point in the map 
    origincol = particle.originIdx(2); % col idx of the original point in the map 
    
    if tmpN >= 0
        atRow = originrow - ceil(tmpN / gridlen - 0.5);
    else
        atRow = originrow + ceil(abs(tmpN) / gridlen - 0.5);
    end
    if tmpE >= 0
        atCol = origincol + ceil(tmpE / gridlen - 0.5);   
    else
        atCol = origincol - ceil(abs(tmpE) / gridlen - 0.5);
    end
end

