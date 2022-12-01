% configure parameters for wheel-slam

paras.datapath       = "data\wheel-ins_DR.bin";
paras.gt_path        = "data\ground_truth.bin";
paras.initheading    = 27.58;  %only for comparison with ground truth

paras.sigmaPhi       = (0.05 * pi/180);
paras.sigmaDis_scale = 0.05;

paras.odomdata_scale    = 0.5;%only process half of the data
paras.NPARTICLES        = 100;
paras.NEFFECTIVE        = paras.NPARTICLES * 0.5;
paras.rollSeqdimension  = 100;
paras.gridlen           = 1.5;
paras.conRevisitNumThr  = 5;
paras.corrCoefThr       = 0.4;
paras.corrCoefNumThr    = 2;