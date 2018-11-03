%% File to merge laps data from RFS/CPG laps
clear all; close all; clc;

%% Load all the files to merge from 
load('exp1_trainingData.mat');
tmpinLat1 = inputParam_lat;
tmpinLong1 = inputParam_long;
tmpoutAcc1 = outputParamAcc_long;
tmpoutDacc1 = outputParamDacc_long;
tmpoutDf1 = outputParamDf_lat;
tmpoutDdf1 = outputParamDdf_lat;
%%
load('exp2_trainingData.mat');
tmpinLat2 = inputParam_lat;
tmpinLong2 = inputParam_long;
tmpoutAcc2 = outputParamAcc_long;
tmpoutDacc2 = outputParamDacc_long;
tmpoutDf2 = outputParamDf_lat;
tmpoutDdf2 = outputParamDdf_lat;
%%
load('exp3_trainingData.mat');
tmpinLat3 = inputParam_lat;
tmpinLong3 = inputParam_long;
tmpoutAcc3 = outputParamAcc_long;
tmpoutDacc3 = outputParamDacc_long;
tmpoutDf3 = outputParamDf_lat;
tmpoutDdf3 = outputParamDdf_lat;
%%
load('exp4_trainingData.mat');
tmpinLat4 = inputParam_lat;
tmpinLong4 = inputParam_long;
tmpoutAcc4 = outputParamAcc_long;
tmpoutDacc4 = outputParamDacc_long;
tmpoutDf4 = outputParamDf_lat;
tmpoutDdf4 = outputParamDdf_lat;
%% 
load('exp5_trainingData.mat');
tmpinLat5 = inputParam_lat;
tmpinLong5 = inputParam_long;
tmpoutAcc5 = outputParamAcc_long;
tmpoutDacc5 = outputParamDacc_long;
tmpoutDf5 = outputParamDf_lat;
tmpoutDdf5 = outputParamDdf_lat;
%%
load('exp6_trainingData.mat');
tmpinLat6 = inputParam_lat;
tmpinLong6 = inputParam_long;
tmpoutAcc6 = outputParamAcc_long;
tmpoutDacc6 = outputParamDacc_long;
tmpoutDf6 = outputParamDf_lat;
tmpoutDdf6 = outputParamDdf_lat;
%% Merge all data now 

inputParam_lat = [tmpinLat1;tmpinLat2;tmpinLat3;tmpinLat4;tmpinLat5;tmpinLat6];
inputParam_long = [tmpinLong1;tmpinLong2;tmpinLong3;tmpinLong4;tmpinLong5;tmpinLong6];
outputParamAcc_long = [tmpoutAcc1;tmpoutAcc2;tmpoutAcc3;tmpoutAcc4;tmpoutAcc5;tmpoutAcc6];
outputParamDacc_long = [tmpoutDacc1;tmpoutDacc2;tmpoutDacc3;tmpoutDacc4;tmpoutDacc5;tmpoutDacc6];
outputParamDf_lat = [tmpoutDf1;tmpoutDf2;tmpoutDf3;tmpoutDf4;tmpoutDf5;tmpoutDf6];
outputParamDdf_lat = [tmpoutDdf1; tmpoutDdf2; tmpoutDdf3; tmpoutDdf4; tmpoutDdf5; tmpoutDdf6];
%%
save('NN_test_trainingDataLatRFS.mat','inputParam_lat','outputParamDf_lat','outputParamDdf_lat');
save('NN_test_trainingDataLongRFS.mat','inputParam_long','outputParamAcc_long','outputParamDacc_long');
%%
