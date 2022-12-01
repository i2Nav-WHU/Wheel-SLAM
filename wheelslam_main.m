addpath("utils\")

config202107311 % config file

n = 100; % run n times
result = zeros(n, 4);

for i = 1:n 
    clc
    disp(['i = ' num2str(i)]);
    [result(i,1:2), result(i,3:4)]= wheelslam_func(paras); 
   
end


path   = paras.datapath;
%write_bin([path 'finalresults_' num2str(paras.NPARTICLES) '.bin'], result);

% fp_res  = fopen([path 'finalresultsall.txt'],'a+');
% s_curt  = datestr(now);
% format longG
% fprintf(fp_res,'%s\n', s_curt);
% fprintf(fp_res,'Particles = %d NEFFECTIVE = %d\n', paras.NPARTICLES, paras.NEFFECTIVE);
% fprintf(fp_res,'Gridlen = %.1f mode = %d\n', paras.gridlen, paras.mode);
% fprintf(fp_res,'Disstd    = %.5f  Phistd  = %.5f \n', paras.sigmaDis_scale, paras.sigmaPhi);
% fprintf(fp_res,'RollSampleDis = %.1f    RollSeqDim = %d\n', paras.sampleDis, paras.rollSeqdimension);
% fprintf(fp_res,'RollSeqWindow = %d  CorrCoefThr = %.1f  CorrCoefNumThr = %d\n',paras.conRevisitNumThr,paras.corrCoefThr,paras.corrCoefNumThr);
% 
% fprintf(fp_res,'%s\n', 'Mean          [Hor. pos.     heading]');
% fprintf(fp_res,'Wheel-SLAM       %.2f          %.2f\n', mean(result(1:i,1)), mean(result(1:i,2)));
% fprintf(fp_res,'Wheel-INS       %.2f        %.2f\n\n', result(1,3), result(1,4));


figure,
subplot(2,1,1),
plot(result(1:i,3), 'LineWidth', 1.5);hold on;plot(result(1:i,1), 'LineWidth', 1.5);
title([num2str(paras.experiencetime) '-' num2str(paras.experiencenum)]);
ylabel('Hor. RMSE(m)'), xlabel('Test No.');box on, grid on;
legend('Wheel-INS','Wheel-SLAM');
set(gca,'fontsize',10,'fontname','Times');
subplot(2,1,2),
plot(result(1:i,4), 'LineWidth', 1.5);hold on;plot(result(1:i,2), 'LineWidth', 1.5);
ylabel('Heading RMSE(deg)'), xlabel('Test No.');box on, grid on;
set(gca,'fontsize',10,'fontname','Times');