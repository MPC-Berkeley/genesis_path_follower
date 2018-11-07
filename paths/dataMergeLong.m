%% File to merge laps data from RFS/CPG laps
clear all; close all; clc;

%% Load all the files to merge from 
load('CPG_day1_test1_trainingData.mat');
tmpinLong1 = inputParam_long;
tmpoutAcc1 = outputParamAcc_long;
tmpoutDacc1 = outputParamDacc_long;
%%
load('NN_test_CPGDay1_RandDataLat10kTrafo2.mat');
tmpinLong2 = inputParam_long;
tmpoutAcc2 = outputParamAcc_long;
tmpoutDacc2 = outputParamDacc_long;
%%
load('CPG_day1_test2_trainingData.mat');
tmpinLong3 = inputParam_long;
tmpoutAcc3 = outputParamAcc_long;
tmpoutDacc3 = outputParamDacc_long;
%%
load('CPG_day1_sim1_trainingData.mat');
tmpinLong4 = inputParam_long;
tmpoutAcc4 = outputParamAcc_long;
tmpoutDacc4 = outputParamDacc_long;
%% 
load('CPG_day1_sim2_trainingData.mat');
tmpinLong5 = inputParam_long;
tmpoutAcc5 = outputParamAcc_long;
tmpoutDacc5 = outputParamDacc_long;

%% Merge all data now 

inputParam_long = [tmpinLong1;tmpinLong2;tmpinLong3;tmpinLong4;tmpinLong5];
outputParamAcc_long = [tmpoutAcc1;tmpoutAcc2;tmpoutAcc3;tmpoutAcc4;tmpoutAcc5];
outputParamDacc_long = [tmpoutDacc1; tmpoutDacc2; tmpoutDacc3; tmpoutDacc4; tmpoutDacc5];
%%
save('NN_test_CPGDay3_TotalDataLongTrafo2.mat','inputParam_long','outputParamAcc_long','outputParamDacc_long');
%%
clc

s_pred = inputParam_long(:, 4:11);
s_pred_norm = s_pred - repmat(inputParam_long(:,1),1,8);

max(s_pred_norm,[],1)
min(s_pred_norm,[],1)

ds_pred = s_pred_norm(:,2:end)-s_pred_norm(:,1:end-1);
max(ds_pred,[],1)
min(ds_pred,[],1)
