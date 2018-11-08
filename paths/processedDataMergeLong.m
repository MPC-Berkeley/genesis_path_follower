%% Merge processed data after all Opt_vals and Duals are synthesized 
%  Merging random data and processed trajectory data
%  To be run AFTER dataMerge is run. All raw trajectory data is saved.
%  Then that is processed through extractDual.jl and saved as a .mat.
%  Make sure all data is normalized!!!!

clear all; close all; clc;

%%
% load('NN_test_CPGDay4_RandTrainingDataLong10k.mat')
% tmpinLong1 = inputParam_long;
% tmpoutDacc1 = outputParamDacc_long;
% tmpoutDual1 = outputParamDual_long;
% tmpoutOpt1 = optVal_long; 

load('NN_test_CPGDay3_RandTrainingDataLong10k.mat')
tmpinLong1 = inputParam_long;
tmpoutDacc1 = outputParamDacc_long;
tmpoutDual1 = outputParamDual_long;
tmpoutOpt1 = optVal_long; 

%% 
load('CPG_day1_test1_trainingDataNormalized.mat')
tmpinLong2 = inputParam_long;
tmpoutDacc2 = outputParamDacc_long;
tmpoutDual2 = outputParamDual_long;
tmpoutOpt2  = optVal_long; 

%% 
% load('CPG_day1_test2_trainingDataNormalized.mat')
% tmpinLong3 = inputParam_long;
% tmpoutDacc3 = outputParamDacc_long;
% tmpoutDual3 = outputParamDual_long;
% tmpoutOpt3  = optVal_long; 
% 

%% Merge
inputParam_long = [tmpinLong1; tmpinLong2]; % tmpinLong3];
outputParamDacc_long = [tmpoutDacc1; tmpoutDacc2]; %tmpoutDacc3];
outputParamDual_long = [tmpoutDual1; tmpoutDual2]; % tmpoutDual3];
optVal_long = [tmpoutOpt1; tmpoutOpt2]; % tmpoutOpt3];

%% Save this crap 

save('NN_test_CPGDay4_BadRand10kAndOneTrajTrainingDataLong.mat','inputParam_long','outputParamDacc_long','outputParamDual_long','optVal_long')
