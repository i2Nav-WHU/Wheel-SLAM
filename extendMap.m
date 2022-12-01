function [particle, atRow, atCol]= extendMap(particle, gridlen)
%If current position is out of the exist grid map, extend the map
    
    extendStep         = 20; %How many rows/cols be added every time. 
    [atRow, atCol]     = getCurrentMapIndex(particle, gridlen); %Get the idx of the vehicle in the current grid map according to its current position
    [curRows, curCols] = size(particle.gridmap);
    rowBlock           = zeros(extendStep, curCols);
    colBlock           = zeros(curRows + extendStep, extendStep);
    
    % Add matrix block to the grid map at a propriate position (depend on the vehicle direction) every time.
    if (atRow > curRows)
        if (atCol >= particle.preIdxinMap(2))
            particle.gridmap        = [particle.gridmap;rowBlock];
            particle.gridmap        = [particle.gridmap colBlock];
        else
            particle.gridmap        = [particle.gridmap;rowBlock];
            particle.gridmap        = [colBlock particle.gridmap];
            particle.originIdx(2)   = particle.originIdx(2) + extendStep;
            particle.preIdxinMap(2) = particle.preIdxinMap(2) + extendStep;
            atCol                   = atCol + extendStep;
        end
    elseif (atRow <= 0)
        if (atCol >= particle.preIdxinMap(2))
            particle.gridmap        = [rowBlock;particle.gridmap];
            particle.gridmap        = [particle.gridmap colBlock];
            particle.originIdx(1)   = particle.originIdx(1) + extendStep;
            particle.preIdxinMap(1) = particle.preIdxinMap(1) + extendStep;
            atRow                   = atRow + extendStep;
        else
            particle.gridmap        = [rowBlock;particle.gridmap];
            particle.gridmap        = [colBlock particle.gridmap];
            particle.originIdx(1)   = particle.originIdx(1) + extendStep;
            particle.originIdx(2)   = particle.originIdx(2) + extendStep;
            particle.preIdxinMap(1) = particle.preIdxinMap(1) + extendStep;
            particle.preIdxinMap(2) = particle.preIdxinMap(2) + extendStep;
            atCol                   = atCol + extendStep;
            atRow                   = atRow + extendStep;
        end
    else
        if (atCol > curCols)
            if (atRow >= particle.preIdxinMap(1))
                particle.gridmap        = [particle.gridmap;rowBlock];
                particle.gridmap        = [particle.gridmap colBlock];
            else
                particle.gridmap        = [rowBlock;particle.gridmap];
                particle.gridmap        = [particle.gridmap colBlock];
                particle.originIdx(1)   = particle.originIdx(1) + extendStep;
                particle.preIdxinMap(1) = particle.preIdxinMap(1) + extendStep;
                atRow                   = atRow + extendStep;
            end
        elseif (atCol <= 0)
            if (atRow >= particle.preIdxinMap(1))
                particle.gridmap        = [particle.gridmap;rowBlock];
                particle.gridmap        = [colBlock particle.gridmap];
                particle.originIdx(2)   = particle.originIdx(2) + extendStep;
                particle.preIdxinMap(2) = particle.preIdxinMap(2) + extendStep;    
                atCol                   = atCol + extendStep;
            else
                particle.gridmap        = [rowBlock;particle.gridmap];
                particle.gridmap        = [colBlock particle.gridmap];
                particle.originIdx(1)   = particle.originIdx(1) + extendStep;
                particle.originIdx(2)   = particle.originIdx(2) + extendStep;
                particle.preIdxinMap(1) = particle.preIdxinMap(1) + extendStep;
                particle.preIdxinMap(2) = particle.preIdxinMap(2) + extendStep;
                atRow                   = atRow + extendStep;
                atCol                   = atCol + extendStep;
            end
        end
    end
    
end

