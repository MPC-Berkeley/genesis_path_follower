%% File to merge laps data from RFS/CPG laps
clear all; close all; clc;

%% Load all the files to merge from 
load('CPG_day1_test1_trainingData.mat');
tmpinLong1 = inputParam_long;
tmpoutAcc1 = outputParamAcc_long;
tmpoutDacc1 = outputParamDacc_long;

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
inputParam_long = [tmpinLong1; tmpinLong3; tmpinLong4; tmpinLong5];
outputParamAcc_long = [tmpoutAcc1; tmpoutAcc3; tmpoutAcc4; tmpoutAcc5];
outputParamDacc_long = [tmpoutDacc1; tmpoutDacc3; tmpoutDacc4; tmpoutDacc5];


% remove s0 and normalize w.r.t. s0
s0 = inputParam_long(:,1);
v0 = inputParam_long(:,2);
u0 = inputParam_long(:,3);
s_ref_old = inputParam_long(:,4:11);
v_ref = inputParam_long(:,12:end);
s_ref = s_ref_old - repmat(s0,1,8);

inputParam_long = [v0 u0 s_ref v_ref];


%%
% save('NN_test_CPGDay3_TotalDataLongTrafo2.mat','inputParam_long','outputParamAcc_long','outputParamDacc_long');

%% just for INFO to generate bounds in Julia
clc

% s_pred = inputParam_long(:, 3:10);
% s_pred_norm = s_pred - repmat(inputParam_long(:,1),1,8);

max(s_ref,[],1)
min(s_ref,[],1)

ds_ref = s_ref(:,2:end)-s_ref(:,1:end-1);
max(ds_ref,[],1)
min(ds_ref,[],1)

%%
clc

% v_pred = inputParam_long(:, 12:end);
max(v_ref,[],1)
min(v_ref,[],1)

dv_ref = v_ref(:,2:end) - v_ref(:,1:end-1);
max(dv_ref,[],1)
min(dv_ref,[],1)