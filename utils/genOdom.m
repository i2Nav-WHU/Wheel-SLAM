%% Generate the odometry and roll sequence data for Wheel-SLAM
clear
close all
imuplace   = 'leftrear';
expdate    = '20210731';
expnum     = '3';
drift      = 'drift';
out_freq   = 10;
rollSeqLen = 50;
odomCols   = rollSeqLen + 6;

start_t       = 557631.374;      %prefered start time for Wheel-SLAM
start_heading = DEG2RAD(359.82); %initial heading got from ground truth
sampleDis     = 0.5;             %prefered sample distance of roll sequence

% read the results from Wheel-INS
wheelins   = read_bin(['E:\WM-IMU\experience\' expdate '\Wheel-IMU\' imuplace '\' expnum '\output\C1\odo-21-S-NAV.bin' drift],23)';
odomda     = read_bin(['E:\WM-IMU\experience\' expdate '\Wheel-IMU\' imuplace '\' expnum '\output\C1\wheelslam_200_' num2str(rollSeqLen) drift '.bin'], odomCols)';


%% generate both the odometry data and roll sequence sampled by equal-distance 
dalenth = size(wheelins,1);

predis = 0;
curdis = 0;
prepos = [odomda(1,2) odomda(1,3)];
curpos = [0 0];
n = 1;

preroll = wheelins(1,9); curroll = wheelins(1,9);
preheading = wheelins(1,10); curheading = wheelins(1,10);lastsampleheading = wheelins(1,10);
pret = wheelins(1,1); curt = wheelins(1,1);

for i=2:dalenth
    if(odomda(i-1,1)<start_t && odomda(i,1)>start_t)
 
        %Interp the start point.
        wpre = (odomda(i,1)-start_t)/(odomda(i,1)-odomda(i-1,1));
        wcur = (start_t-odomda(i-1,1))/(odomda(i,1)-odomda(i-1,1));
        prepos = odomda(i-1,2:3);
        curpos = odomda(i,2:3);
        postmp  = [prepos(1)*wpre + curpos(1)*wcur, prepos(2)*wpre + curpos(2)*wcur];
        rolltmp = wheelins(i-1,9)*wpre + wheelins(i,9)*wcur;
        
        preheading = wheelins(i-1,10);
        curheading = wheelins(i,10);
        headingtmp = preheading*wpre + curheading*wcur;
        if(preheading - curheading > pi)
            headingtmp = headingtmp + 2*pi*wcur;
        elseif(preheading - curheading < -pi)
            headingtmp = headingtmp + 2*pi*wpre;
        end
        
        prepos = postmp;
        curpos = postmp;
        preheading = start_heading;
        curheading = start_heading;
        lastsampleheading = headingtmp;
        pret = start_t; 
        curt = start_t;
        
        res(n,:) = [start_t 0 start_heading rolltmp];
        break;
    end
end

%get odometry informatin from the start point
while i<=dalenth

    
    curpos = [odomda(i,2) odomda(i,3)];%The position from DR is more continuous and without jags, thus we must get the vehicle position from odomda instead of wheelins! or it may cause scale drift.
    
    curdis = curdis + sqrt((prepos(2) - curpos(2))^2 + (prepos(1) - curpos(1))^2);
    curroll = wheelins(i,9);
    curheading = wheelins(i,10);
    curt = wheelins(i,1);
 
    
    if (predis<sampleDis && curdis>sampleDis)
        
        n = n + 1;
		wpre = (curdis-sampleDis)/(curdis-predis);
        wcur = (sampleDis-predis)/(curdis-predis);
        postmp  = [prepos(1)*wpre + curpos(1)*wcur, prepos(2)*wpre + curpos(2)*wcur];
        

        rolltmp = preroll*wpre + curroll*wcur;
        
%         headingtmp = preheading*wpre + curheading*wcur;
%         if(preheading - curheading > pi)
%             headingtmp = headingtmp + 2*pi*wcur;
%         elseif(preheading - curheading < -pi)
%             headingtmp = headingtmp + 2*pi*wpre;
%         end
        ttmp = pret*wpre + curt*wcur;
        
        headingtmp = atan2(postmp(2)-prepos(2), postmp(1)-prepos(1));
        
        headingInc = headingtmp - lastsampleheading;
        if(headingInc > pi)
            headingInc = headingInc - 2*pi;
        elseif(headingInc < -pi)
            headingInc = headingInc + 2*pi;
        end
        
        res(n,:) = [ttmp sampleDis headingInc rolltmp];
        curdis = 0;
        
        curpos  = postmp;
        curroll = rolltmp;
        curt = ttmp;
        curheading = headingtmp;
        i = i-1;
        lastsampleheading = headingtmp;
    end
    
    predis = curdis;
    prepos = curpos;
    preroll = curroll;
    preheading = curheading;
    pret = curt;
    i = i+1;
end
figure,plot(res(:,1),res(:,4),'LineWidth', 1.5);hold on;

Wc = 2*1/(25/sampleDis);
[b, a] = butter(4, Wc);%butterworth 低通滤波参数
res(:, 4) = filtfilt(b, a, res(:, 4));%butterworth 低通滤波
plot(res(:,1),res(:,4),'LineWidth', 1.5);

% t_tmp = abs(res(:,1) - start_t);
% idx = find(t_tmp==min(min(t_tmp)));

% res(1:idx-1,:) = [];
% res(1,2:4) = [0,0, DEG2RAD(start_heading)];
% plot(res(:,1),res(:,4),'LineWidth', 1.5);

write_bin(['E:\WM-IMU\experience\' expdate '\Wheel-IMU\' imuplace '\' expnum '\output\C1\dis' num2str(sampleDis) drift '.bin'], res);

%% check the generated traj.
DRres = zeros(size(res));
DRres(1,:) = [res(1,1), 0, 0, res(1,3)];
pre_state = DRres(1,:);
for i = 2:size(res, 1)
    t         = res(i,1);
    Dis       = res(i,2);
    dphi      = res(i,3);
    phi       = dphi/2 + pre_state(4);
    phi       = pi_to_pi(phi);
    DRres(i,:) = [t pre_state(2) + Dis*cos(phi) pre_state(3) + Dis*sin(phi) pi_to_pi(pre_state(4) + dphi)];
    pre_state  = DRres(i,:);
end

rtk = read_bin(['E:\WM-IMU\experience\' expdate '\reference\gins.bin'],11)';
label     = find(diff(rtk(:,2)) ~= 0);
rtk  = rtk(label+1,:);
rtkInt = [DRres(:,1) interp1(rtk(:,2),rtk(:,3:5), DRres(:,1)) interp1_Azimuth(rtk(:,2), DEG2RAD(rtk(:,11)), DRres(:,1))];
[rtkInt(:,2),rtkInt(:,3),rtkInt(:,4)] = BLH2NED([rtkInt(1,2),rtkInt(1,3),rtkInt(1,4)],[rtkInt(:,2),rtkInt(:,3),rtkInt(:,4)]);
wheelinsInt = [DRres(:,1) interp1(wheelins(:,1), wheelins(:,2:3), DRres(:,1)) interp1_Azimuth(wheelins(:,1), wheelins(:,10), DRres(:,1))];
wheelinsInt(:,2:3) = wheelinsInt(:,2:3) - wheelinsInt(1,2:3);

figure,
plot(rtkInt(:,3), rtkInt(:,2),'LineWidth', 1.5);axis equal;hold on;
plot(wheelinsInt(:,3), wheelinsInt(:,2),'LineWidth', 1.5);hold on;
plot(DRres(:,3), DRres(:,2),'LineWidth', 1.5);
legend('Ground Truth', 'Wheel-INS', 'Wheel-INS DR');
title(['Sample Dis. = ' num2str(sampleDis)]);

wheelins_headingerr   = getheadingerr(rtkInt(:,5), wheelinsInt(:,4));
wheelinsDR_headingerr = getheadingerr(rtkInt(:,5), DRres(:,4));

figure,
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.48]);
plot(DRres(:,1), wheelinsDR_headingerr ,DRres(:,1), wheelins_headingerr,  'LineWidth', 1.5);
xlabel('Time'),ylabel('Heading Error(deg)');
legend('Wheel-INS(DR)', 'Wheel-INS');set(gca,'fontsize',14,'fontname','Times');



%% generate the roll sequences
% wheelins  = [wheelins(:, 1:3) wheelins(:, 9)];
% 
% figure,plot(wheelins(:,1),wheelins(:,4),'LineWidth', 1.5);hold on;
% 
% Wc = 2*5/200;
% [b, a] = butter(4, Wc);%butterworth 低通滤波参数
% wheelins(:, 4) = filtfilt(b, a, wheelins(:, 4));%butterworth 低通滤波
% plot(wheelins(:,1),wheelins(:,4),'LineWidth', 1.5);
% 
% odomlen     = size(odomda, 1);
% fullrollseq = zeros(odomlen, rollSeqLen);
% sampleDis   = 1;
% rollseqtmp  = zeros(1, rollSeqLen);
% 
% for i = 1:odomlen
%     idx = find(abs(wheelins(:,1) - odomda(i,1)) < 0.002);
%     j = idx-1;
%     n = 1;
%     predis = 0.0; curdis = 0.0; 
%     preroll = wheelins(idx,4); curroll = wheelins(idx,4);
%     curpos = [0, 0]; prepos = [wheelins(idx,2), wheelins(idx,3)];
%     rollseqtmp(n) = curroll;
%     while (j > 0)
%         curroll = wheelins(j,4);
%         curpos  = [wheelins(j,2), wheelins(j,3)];
%         curdis  = curdis + sqrt((prepos(2) - curpos(2))^2 + (prepos(1) - curpos(1))^2);
%         
%         prepos  = curpos;
%         
%         if (predis<sampleDis && curdis>sampleDis)
% 		
% 			n = n + 1;
%             if(n>rollSeqLen)
%                 break;
%             end
% 			rollseqtmp(n) = preroll*(curdis-sampleDis)/(curdis-predis) + curroll*(sampleDis-predis)/(curdis-predis);
% 			
% 			curdis = 0.0;
%         end
%         
%         predis = curdis;
% 		preroll = curroll;
%         j = j-1;
%     end
%     
%     if(n < rollSeqLen)
%         fullrollseq(i, :) = zeros(1, rollSeqLen);
%     else
%         fullrollseq(i, :) = rollseqtmp;
%     end
%     
% end
% 
% for i = 240:290
%     figure,
%     plot(fullrollseq(i,5:end), 'LineWidth', 1.5);hold on;
%     plot(fullrollseq(i+1,5:end), 'LineWidth', 1.5);
%     legend('Pre','Cur');
% end

function x = pi_to_pi(x)
if x > pi
    x= x - 2*pi;
elseif x < -pi
    x= x + 2*pi;
end
end