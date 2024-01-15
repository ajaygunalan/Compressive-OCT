clear all; close all; clc;
before = readmatrix("data\getDepthFromSparse3Doct\1\ScanNum_1_Estimation.csv");
after = readmatrix("data\getDepthFromSparse3Doct\1\ScanNum_2_Estimation.csv");
scanNum = "2";
trialNum = "1";
baseFolder = 'C:\Ajay_OCT\OCT-Guided-AutoCALM\data\getDepthFromSparse3Doct\';
prefix = sprintf('%s_', scanNum);
folderLocation = fullfile(baseFolder, trialNum);
depth = depthEstimation(after, before,prefix, folderLocation);