function [err1, rms1, err2, rms2, err_p, rms_p] = ResCompare(wheelslam, wheelslam_p, wheelinsDR, gt_path, interpfreq)
    
    %wheelslam = read_bin(filepath1, 4)';
    %wheelins  = read_bin(filepath2, 4)';
    wheelins  = wheelinsDR;
    RTK_data  = read_bin(gt_path, 11)'; 
    label     = find(diff(RTK_data(:,2)) ~= 0);
    RTK_data  = RTK_data(label+1,:);
    RTK_local = RTK_data;
    proj_t    = find(abs(RTK_local(:,2)-wheelslam(1,1)) < 0.0025, 1);

    if(~isempty(proj_t))
        [RTK_local(:,3),RTK_local(:,4),RTK_local(:,5)] = BLH2NED([RTK_data(proj_t,3),RTK_data(proj_t,4),RTK_data(proj_t,5)],[RTK_data(:,3),RTK_data(:,4),RTK_data(:,5)]);
    else
        disp('Reference & Wheel-INS Time Error!');
        return;
    end    

    start_t    = max([wheelins(1,1) wheelslam(1,1) RTK_local(1,2)]);
    end_t      = min([wheelins(end,1) wheelslam(end,1) RTK_local(end,2)]);
    common_t   = (start_t:(1/interpfreq):end_t)';
    compt      = common_t - common_t(1,1);
    
    RTK_heading       = interp1_Azimuth(RTK_local(:,2),DEG2RAD(RTK_local(:,11)),common_t);
    wheelins_heading  = interp1_Azimuth(wheelins(:,1),wheelins(:,4),common_t); 
    wheelslam_heading = interp1_Azimuth(wheelslam(:,1),wheelslam(:,4),common_t);
    
    wheelins_headingerr  = getheadingerr(RTK_heading, wheelins_heading); 
    wheelslam_headingerr = getheadingerr(RTK_heading, wheelslam_heading);

    wheelins   = [common_t interp1(wheelins(:,1),wheelins(:,2:end),common_t)];
    wheelslam  = [common_t interp1(wheelslam(:,1),wheelslam(:,2:end),common_t)];
    RTK_local  = [common_t interp1(RTK_local(:,2),RTK_local(:,3:5),common_t)];
    wheelslam_p = [common_t interp1(wheelslam_p(:,1),wheelslam_p(:,2:end),common_t)];

    err1  = [common_t RTK_local(:,2:3) - wheelslam(:,2:3) wheelslam_headingerr];
    err2  = [common_t RTK_local(:,2:3) - wheelins(:,2:3) wheelins_headingerr];
    err_p = [common_t RTK_local(:,2:3) - wheelslam_p(:,2:3) ];
    
    horposerr1 = sqrt(err1(:, 2).*err1(:, 2) + err1(:, 3).*err1(:, 3));
    horposerr2 = sqrt(err2(:, 2).*err2(:, 2) + err2(:, 3).*err2(:, 3));
    
    rms1  = [rms(horposerr1) rms(err1(:, 4))];
    rms2  = [rms(horposerr2) rms(err2(:, 4))];
    rms_p = rms(sqrt(err_p(:, 2).*err_p(:, 2) + err_p(:, 3).*err_p(:, 3)));

    
%     rms1  = [rms(err1(:, 2)) rms(err1(:, 3)) rms(err1(:, 4))];
%     rms2  = [rms(err2(:, 2)) rms(err2(:, 3)) rms(err2(:, 4))];
%     rms_p = [rms(err_p(:, 2)) rms(err_p(:, 3))];
    
    
%% plot traj.
    figure,
    plot(wheelins(:,3),wheelins(:,2), 'LineWidth', 1.5);
    set(gca,'fontsize',10,'fontname','Times');
    xlabel('East(m)'), ylabel('North(m)'),grid on,axis equal,box on;hold on;
    plot(wheelslam(:,3),wheelslam(:,2), 'LineWidth', 1.5);
    plot(RTK_local(:,3),RTK_local(:,2),'LineWidth', 1.5);hold on;
    set(gca,'fontsize',10,'fontname','Times');
    legend('Wheel-INS', 'Wheel-SLAM', 'Ground Truth');
% %     export_fig([filepath  'trajcomp'],'-tif','-transparent','-m4');
% %     export_fig([filepath  'trajcomp'],'-png','-transparent','-m4');
% 
%% plot horizontal pos. RMS and heading err.

%     figure,
%     subplot(2,1,1); 
%     plot(compt, horposerr2, 'LineWidth', 1.5);hold on;
%     plot(compt, horposerr1, 'LineWidth', 1.5);
%     legend('Wheel-INS','Wheel-SLAM');
%     set(gca,'fontsize',10,'fontname','Times');
%     ylabel('Hor. RMSE(m)'), box on, grid on;
%     subplot(2,1,2);
%     plot(compt, wheelins_headingerr, 'LineWidth', 1.5);hold on;
%     plot(compt, wheelslam_headingerr, 'LineWidth', 1.5);
%     xlabel('Time(s)'),ylabel('Heading Error(deg)');
%     set(gca,'fontsize',10,'fontname','Times');

%% Plot North and East Pos. RMS Err.
%     figure,
%     subplot(2,1,1); plot(compt, err2(:,2), 'LineWidth', 1.5);
%     hold on;plot(compt, err1(:,2), 'LineWidth', 1.5);
% %     hold on;plot(compt, err_p(:,2), 'LineWidth', 1.5);
%     legend('Wheel-INS','Wheel-SLAM');
%     set(gca,'fontsize',10,'fontname','Times');
%     ylabel('North error(m)'), box on, grid on;
%     subplot(2,1,2); plot(compt, err2(:,3), 'LineWidth', 1.5);
%     hold on;plot(compt, err1(:,3), 'LineWidth', 1.5);
% %     hold on;plot(compt, err_p(:,3), 'LineWidth', 1.5);
%     set(gca,'fontsize',10,'fontname','Times');
%     ylabel('East error(m)'),xlabel('Time(s)'), box on, grid on; 
% %     export_fig([filepath  'poserr'],'-tif','-transparent','-m4');
% %     export_fig([filepath  'poserr'],'-png','-transparent','-m4');
% 
%% plot heading err.
%     figure,
%     subplot(2,1,1);
%     plot(compt, wheelslam_heading, 'LineWidth', 1.5);hold on;
%     plot(compt, RTK_heading, 'LineWidth', 1.5);
%     legend('Wheel-SLAM','Ground Truth');
%     xlabel('Time(s)'),ylabel('Heading(deg)');
%     subplot(2,1,2);
%     plot(compt, wheelins_headingerr, 'LineWidth', 1.5);hold on;
%     plot(compt, wheelslam_headingerr, 'LineWidth', 1.5);
%     legend('Wheel-INS','Wheel-SLAM');
%     xlabel('Time(s)'),ylabel('Heading Error(deg)');
%     set(gca,'fontsize',10,'fontname','Times');
% 
% %     export_fig([filepath  'headingerr'],'-tif','-transparent','-m4');
% %     export_fig([filepath  'headingerr'],'-png','-transparent','-m4');
% 
%% plot 3D figure
%     
%     figure,
%     plot3(wheelslam(:,3),wheelslam(:,2), compt, 'LineWidth', 1.5);
%     set(gca,'fontsize',10,'fontname','Times');
%     xlabel('East(m)'), ylabel('North(m)'),grid on,box on;
%     legend( 'Wheel-INS');
%     figure,
%     plot3(RTK_local(:,3),RTK_local(:,2), compt, 'LineWidth', 1.5);
%     set(gca,'fontsize',10,'fontname','Times');
%     xlabel('East(m)'), ylabel('North(m)'),grid on,box on;
%     legend('Ground Truth');

end
