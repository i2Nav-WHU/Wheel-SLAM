function [particle, revisitGrids] = updateGridmap(particle, curAllGrids, curAllRollSeq, curallheading, revisit_tThr, idx)
%INPUT:
%idx:the index of the current measurement in the full odometry file

newgrids     = [];
j            = 1;
k            = 1;
newMapattr   = [];
revisitGrids = [];
for i = 1:size(curAllGrids, 1)

    if particle.gridmap(curAllGrids(i, 1), curAllGrids(i, 2)) == 0
        newgrids(j, :) = [curAllGrids(i, :) curAllRollSeq(i, :) curallheading(i,:) idx];
        j              = j+1;
        
    else
        if (particle.t - particle.mapattr(particle.gridmap(curAllGrids(i, 1), curAllGrids(i, 2))).last_vt > revisit_tThr)
            % if the time inteval between two visit is enough
            revisitGrids(k,:) = [curAllGrids(i, :) curAllRollSeq(i, :) idx];% grid index + current roll measurement + index in the full roll file
            k                 = k + 1;
        end
    end
end
% add new grids
if ~isempty(newgrids)
    
    for i = 1:size(newgrids, 1)
        particle.gridmap(newgrids(i, 1), newgrids(i, 2)) = particle.totalGrids + i;
        tmpgridattr.Num          = 1;
        tmpgridattr.visited      = 1; 
        tmpgridattr.revisit      = 0;
        tmpgridattr.last_vt      = particle.t;
        tmpgridattr.Value        = newgrids(i, 3:3+size(curAllRollSeq,2)-1);
        tmpgridattr.enterheading = newgrids(i, end-1);
        tmpgridattr.idx          = idx;
        newMapattr               = [newMapattr;tmpgridattr];
    end
    particle.totalGrids = particle.totalGrids + size(newgrids, 1);
    particle.mapattr    = [particle.mapattr;newMapattr];
end

end

