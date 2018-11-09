%% Merge processed data after all Opt_vals and Duals are synthesized 
%  Merging random data and processed trajectory data
%  To be run AFTER dataMerge is run. All raw trajectory data is saved.
%  Then that is processed through extractDual.jl and saved as a .mat.
%  Make sure all data is normalized!!!!

clear all; close all; clc;

%%
load('NN_test_CPGDay4_SmoothVCRandTrainingDataLat10k.mat')
tmpinlat1 = inputParam_lat;
tmpoutDdf1 = outputParamDdf_lat;
tmpoutDual1 = outputParamDual_lat;
tmpoutOpt1 = optVal_lat; 

%% 
load('CPG_day1_test1_latTrainingDataVCnormalized.mat')
tmpinlat2 = inputParam_lat;
tmpoutDdf2 = outputParamDdf_lat;
tmpoutDual2 = outputParamDual_lat;
tmpoutOpt2  = optVal_lat; 

%% 
% load('CPG_day1_test2_latTrainingDataVCnormalized.mat')
% tmpinlat3 = inputParam_lat;
% tmpoutDdf3 = outputParamDdf_lat;
% tmpoutDual3 = outputParamDual_lat;
% tmpoutOpt3  = optVal_lat; 
%% 
% load('CPG_day1_sim1_latTrainingDataVCnormalized.mat')
% tmpinlat4 = inputParam_lat;
% tmpoutDdf4 = outputParamDdf_lat;
% tmpoutDual4 = outputParamDual_lat;
% tmpoutOpt4  = optVal_lat; 
%% Merge
inputParam_lat = [tmpinlat1; tmpinlat2]; %tmpinlat3]; % tmpinlat4]; % tmpinlat3];
outputParamDdf_lat = [tmpoutDdf1; tmpoutDdf2]; %tmpoutDdf3]; %tmpoutDdf4]; %tmpoutDdf3];
outputParamDual_lat = [tmpoutDual1; tmpoutDual2]; %tmpoutDual3]; %tmpoutDual4]; % tmpoutDual3];
optVal_lat = [tmpoutOpt1; tmpoutOpt2]; %tmpoutOpt3]; %tmpoutOpt4]; % tmpoutOpt3];

%% Save this crap 

% save('NN_test_CPGDay4_2sim2testTrajTestDataLat.mat','inputParam_lat','outputParamDdf_lat','outputParamDual_lat','optVal_lat')
save('NN_test_CPGDay4_SmoothVCRand10kOneTrajDataLat.mat','inputParam_lat','outputParamDdf_lat','outputParamDual_lat','optVal_lat')
