function [wheelslam_rms, wheelins_rms] = wheelslam_func(paras)

% clear
% close all
format longG

datapath         = paras.datapath;
rollSeqdimension = paras.rollSeqdimension; %Dimension of the roll sequence measurement
NPARTICLES       = paras.NPARTICLES;
gridlen          = paras.gridlen;      
NEFFECTIVE       = paras.NEFFECTIVE;   %Threshold of the effective particles
sigmaDis_scale   = paras.sigmaDis_scale; % unit: m
sigmaPhi         = paras.sigmaPhi;     % unit: radians
initheading      = paras.initheading; % Initial heading of the vehicle
conRevisitNumThr = paras.conRevisitNumThr;% The number threshold for determining the revisit of the particle
corrCoefThr      = paras.corrCoefThr;
corrCoefNumThr   = paras.corrCoefNumThr;
gt_path          = paras.gt_path;

freq             = 10;
odom             = read_bin(datapath, 4)';% time, dis(equal), headingInc, roll

%Import odometry info from Wheel-INS, including timestamp, dis_increment, 
%heading increment and roll angle

revisit_tThr   = 30;      %Minimal time interval for revisit identification. Sometimes car stop
initmapsize    = 101;
progrssThr     = -100;
start_t        = odom(1,1);
odomsize       = floor(size(odom, 1)*paras.odomdata_scale);

particles      = init_particles(NPARTICLES, initmapsize, odomsize, start_t, initheading, corrCoefNumThr);
res            = zeros(odomsize, 4);
res(1, :)      = [start_t particles(1).xv'];

pre_refstate      = [start_t particles(1).xv'];
wheelinsDR        = zeros(odomsize, 4);
wheelinsDR(1,:)   = pre_refstate;
revisitParticles  = 0;

% This is used to set the randanstream of the particles, so as to
% make the results the same everytime.
% for i = 1:NPARTICLES
%     particles(i).randstream = RandStream.create('mlfg6331_64','Seed', i);
% end
% 
% resample_randstream = RandStream.create('mlfg6331_64','Seed', idx);
%---------------------------------------------------------------------

for i = 2:odomsize

    
    % Current measurement of roll sequence.
    % curRollmeas = odom(i, 5:size(odom, 2));
    curRollmeas     = odom(i, 4);
    ref_state       = state_predict_ref(odom(i, :), pre_refstate);% Get the DR results from the odometry file for comparision.
    wheelinsDR(i,:) = ref_state;
    pre_refstate    = ref_state;
    
    t_s             = clock;
    revisitNum      = 1;
    curodom = odom(i, :);
    
    for j = 1:NPARTICLES

        particles(j)                 = state_predict(particles(j), curodom, sigmaDis_scale, sigmaPhi); %State prediction for every particle.
        [particles(j), atRow, atCol] = extendMap(particles(j), gridlen); %Extend (or not) the grid map maintained by every particle.

        
        if (particles(j).conRevisitNum ~= conRevisitNumThr)
            particles(j).conRevisitNum = particles(j).conRevisitNum + 1;
        else
            % sliding window for loop closure detection.

            particles(j).conCorrCoef(1:(conRevisitNumThr-1)) = particles(j).conCorrCoef(2:conRevisitNumThr);
            particles(j).conCorrCoef(conRevisitNumThr)       = -1;
           
        end

        particles(j).conCorrCoef(particles(j).conRevisitNum)    = -1;
        
        if (atRow == particles(j).preIdxinMap(1) && atCol == particles(j).preIdxinMap(2)...
                && particles(j).lastTrajRevisitIdx < i-1)
            
            % If one step is too short (still in the same grid) and if last step is not a revisit
            numforgrid = particles(j).mapattr(particles(j).gridmap(atRow, atCol)).Num;
            particles(j).mapattr(particles(j).gridmap(atRow, atCol)).Value...
            = (particles(j).mapattr(particles(j).gridmap(atRow, atCol)).Value ...
            * (numforgrid/(numforgrid+1)) + curRollmeas./(numforgrid +1));
           
            % Average the roll measurement in the same grid.            
            particles(j).mapattr(particles(j).gridmap(atRow, atCol)).Num = numforgrid + 1;

        else
        
            innerGridsIndex  = [];
            revisitGrids     = [];
            allRollSeq       = [];
            allheading       = [];

            curallheading = [allheading;particles(j).xv(3)];
            curAllRollSeq = [allRollSeq;curRollmeas];  
            curGrid       = [atRow, atCol];
            curAllGrids   = [innerGridsIndex; curGrid];
            curAllGrids   = unique(curAllGrids, 'rows','stable');
            
            if (~isempty(curAllGrids))

                %Update (or not) the grid map maintained by every particle
                %and get the revisit grids after revisit time interval check. 
                %Check if current grid is a new grid.
                [particles(j), revisitGrids] = updateGridmap(particles(j), curAllGrids, curAllRollSeq, curallheading, revisit_tThr, i);

            end

            if (~isempty(revisitGrids))
               
                for ii = 1:size(revisitGrids, 1)
                    %Threshold for the heading difference between current entance and last one
                    headingdiff = abs(getheadingdiff(particles(j).xv(3), particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).enterheading));
                    if(headingdiff > (pi/6))
                        break;
                    end
                        
                    particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).revisit = ...
                    particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).revisit + 1;
                    particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).Num = ...
                    particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).Num + 1;
                   
                    %Get the historical index of the revisited grid in the
                    %roll sequence.
                    revisitIdx = particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).idx;
                    if( revisitIdx < rollSeqdimension)
                        break;
                    else
                        particles(j).lastTrajRevisitIdx = i; %where the particle detected a revisit by the traj.
                        
                        currollseq = odom(i-rollSeqdimension+1:i,4);% Current roll sequence from the revisited grid
                        maprollseq = odom(revisitIdx-rollSeqdimension+1:revisitIdx,4); %Historical roll sequence
                        corrMat    = corrcoef(currollseq, maprollseq);
                        corrvalue  = corrMat(1,2); % Get the correlation value                            
                        particles(j).conCorrCoef(particles(j).conRevisitNum) = corrvalue;
                        corrCoefNumtmp = length(particles(j).conCorrCoef(particles(j).conCorrCoef> corrCoefThr));

                        if(particles(j).conRevisitNum == conRevisitNumThr && ...
                           corrCoefNumtmp > corrCoefNumThr &&...
                           corrvalue > corrCoefThr) 
                           %criteria 1. Full of the correlation window;
                           %2.Enough value in the window larger than the
                           %threshold
                           %3.Current correlation value larger than the
                           %threshold;

                            tmpSeq = particles(j).conCorrCoef((particles(j).conCorrCoef > corrCoefThr ));

                            particles(j).w = particles(j).w * exp(rms(tmpSeq)* (corrCoefNumtmp/conRevisitNumThr)); %Update the weight of the particles reported a convinced loop closure
                            particles(j).mapattr(particles(j).gridmap(revisitGrids(ii,1), revisitGrids(ii,2))).last_vt = particles(j).t;
                            particles(j).totalrevisitNum = particles(j).totalrevisitNum + 1;
                            particles(j).revisitpos(particles(j).totalrevisitNum, :) = [particles(j).t particles(j).xv(1) particles(j).xv(2)];

                        end

                        % count the revisited particle only once
                        if (ii == 1)
                            revisitParticles(revisitNum) = j;
                            revisitNum = revisitNum + 1;
                        end
                        
                    end

                end

            end
            
        end

        particles(j).preIdxinMap = [atRow, atCol];
        particles(j).prepos      = particles(j).xv;
        particles(j).traj(i,:)   = (particles(j).xv)';

    end

    % Check if resample is needed, if yes, do it. Generally this function
    % needn't to be modified, we can set the parameters "NEFFECTIVE"
    particles    = resampleParticles(particles, NEFFECTIVE);%, resample_randstream
    %particles    = resampleParticles(particles, NEFFECTIVE, resample_randstream);

    %save system output
    w            = [particles.w];
    xv           = [particles.xv];
    if (mean(abs(xv(3,:))) > abs(mean(abs(xv(3,:))) - pi))
        xv(3, :)     = zero_to_2pi(xv(3, :));
    else
        xv(3, :)     = pi_to_pi(xv(3, :));
    end
    
    ii       = find(w == max(w),1);
    w        = w/sum(w);
    xvmean   = [mean(sum(w.* xv(1, :))) mean(sum(w.* xv(2, :))) mean(sum(w.* xv(3, :)))];
    res(i,:) = [particles(1).t xvmean];
    
    revisitParticles = 0;
    
    runProgress  = floor((i-1)/(odomsize) * 1000);
    if(runProgress > progrssThr)
        clc;
        progrssThr = runProgress;
        disp(['******Wheel SLAM RUNNING:' num2str(0.1*runProgress) '%******']);
    end
   
end


% figure,
% plot3(res(:,3), res(:,2), res(:,1)-res(1,1), 'LineWidth', 1.5),grid on;
% title('Weighted mean traj.');hold on;
% plot(res(1,3), res(1,2), 'go');hold on;
% plot(res(end,3), res(end,2), 'ro');
% set(gca,'fontsize',10,'fontname','Times');

% plotMap(particles(ii),i, path);


% write_bin(resfile, res);
% write_bin(wheelinsDRpath, wheelinsDR);

particleres = [res(:, 1) particles(ii).traj];% trajectory of the particle with highest weight
[wheelslam_err, wheelslam_rms, wheelins_err, wheelins_rms, wheelslam_err_p, wheelslam_rms_p] =...
    ResCompare(res, particleres, wheelinsDR, gt_path, freq);

%% plot the total revisit num of all the particles
% revisitNums           = [particles.totalrevisitNum];
% figure,
% set(gcf,'unit','normalized','position',[0.05,0.05,0.64,0.48]);
% plot(revisitNums,'*'); set(gca,'fontsize',10,'fontname','Times');
% title('Total Revisit Num');
% figure,
% plot3(particles(ii).revisitpos(:,3), particles(ii).revisitpos(:,2), particles(ii).revisitpos(:,1),'*');
% title(['Particle =' num2str(ii) 'idx = ' num2str(idx)]);

% save particle data
% save([path 'particle_' num2str(ii) '_' s_curt], 'particles');


% format longG
% fprintf(fp_res,'%s\n', s_curt);
% fprintf(fp_res,'Particles = %d NEFFECTIVE = %d\n', NPARTICLES, NEFFECTIVE);
% fprintf(fp_res,'Gridlen = %.1f mode = %d\n', gridlen, mode);
% fprintf(fp_res,'Disstd    = %.5f  Phistd  = %.5f \n', sigmaDis_scale, sigmaPhi);
% fprintf(fp_res,'RollSampleDis = %.1f    RollSeqDim = %d\n', paras.sampleDis, rollSeqdimension);
% fprintf(fp_res,'RollSeqWindow = %d  CorrCoefThr = %.1f  CorrCoefNumThr = %d\n',conRevisitNumThr,corrCoefThr,corrCoefNumThr);
% fprintf(fp_res,'Start_t = %f  TotalData_t = %f  Process_t = %s s\n',start_t, totaldata_t, processtime);
% fprintf(fp_res,'%s\n', 'RMS        [North     East]');
% fprintf(fp_res,'WheelSLAM    %.5f  %.5f\n', wheelslam_rms(1), wheelslam_rms(2));
% fprintf(fp_res,'WheelSLAM_p  %.5f  %.5f\n', wheelslam_rms_p(1), wheelslam_rms_p(2));
% fprintf(fp_res,'WheelINS     %.5f  %.5f\n', wheelins_rms(1), wheelins_rms(2));
% fprintf(fp_res,'Heading RMSE:  Wheel-SLAM:%.5f Wheel-INS:%.5f\n\n', wheelslam_rms(3), wheelins_rms(3));

end

function p = init_particles(np, mapsize, odomsize, start_t, initheading, corrCoefNumThr)
   
    %Grid map attribute
    gridattr.Num     = 1; %how many times has this grid been visited
    gridattr.visited = 1; %if this grid has been visited
    gridattr.revisit = 0; %if this grid has been revisited
    gridattr.last_vt = start_t;%last visit time of the grid

    gridattr.Value   = zeros(1, 1);%only single roll angle
    gridattr.idx     = 1;% the index of the current roll measurement in the full odometry file
    initheadingrad   = DEG2RAD(initheading);
    gridattr.enterheading = initheadingrad;%the heading of the vehicle when it entering this grid
    
    
    p.conRevisitNum  = 0;%length of the correlation sliding window
    p.conCorrCoef    = zeros(1,corrCoefNumThr);
    p.gridmap        = zeros(mapsize, mapsize);% save the idx in the grid attibute matrix at every grid of the map
    %the coordinates of the original point is (0,0), but it is at the
    %central of the map
    p.originIdx      = [(mapsize+1)/2, (mapsize+1)/2];% the idx of the original point in the grid map (which would be updated with the extension of the map).
    p.w              = 1/np;          %weight
    p.xv             = [0;0;initheadingrad]; %state: N, E, Phi

    p.prepos         = [0;0;initheadingrad]; %last position and heading of the vehicle
    p.preIdxinMap    = p.originIdx; %last idx of the vehicle in the grid map
    p.t              = start_t;

    p.totalrevisitNum = 0; %how many convinced loop closure proposed by this particle
    p.revisitpos      = zeros(1,3);%The positions of the vehicle where the convinced loop closure proposed by this particle
    
    
    p.mapattr(1)     = gridattr;% attibute of the first grid in the map. A large grid attribute matrix would be built with the moving of the vehicle
    p.totalGrids     = 1;%
    p.gridmap(p.originIdx(1), p.originIdx(2)) = 1;% grid map maintaind by every particle
    p.traj           = zeros(odomsize,3);% the trajectory of the vehicle estimated by this particle
    p.traj(1,:)      = (p.xv)';
    p.lastTrajRevisitIdx = 0;%The Idx in the odom file where the particle detects a revisit by its traj, (no roll needed)
    p                = repmat(p, [np 1]);
    
end

function x_vec = zero_to_2pi(x_vec)
    lenth = length(x_vec);
    for i = 1:lenth
        if(x_vec(i)<0)
            x_vec(i) = x_vec(i) + 2*pi;
        end
    end
    
end

function x = pi_to_pi(x)
    if x > pi
        x= x - 2*pi;
    elseif x < -pi
        x= x + 2*pi;
    end
end

function hdiff = getheadingdiff(heading1, heading2)
    hdiff = heading1 - heading2;
    if hdiff > pi
        hdiff= hdiff - 2*pi;
    elseif hdiff < -pi
        hdiff= hdiff + 2*pi;
    end
end

function [bestIdx, lagestcorrValue] = getBestMacth(currollseq, revisitIdx, searchRad, odom, rollSeqdimension)

    lagestcorrValue = -exp(5);
    for i = (revisitIdx-searchRad):(revisitIdx+searchRad)
        maprollseq = odom(i-rollSeqdimension+1:i, 4);
        corrMat    = corrcoef(currollseq, maprollseq); 
        corrvalue  = (corrMat(1,2) + corrMat(2, 1))/2;
        if corrvalue > lagestcorrValue
            lagestcorrValue = corrvalue;
            bestIdx = i;
        end
    end
end

