%% File to merge laps data from RFS/CPG laps
clear all; close all; clc;

%% Load all the files to merge from 
load('CPG_day1_test1_trainingData.mat');
tmpinLat1 = inputParam_lat;
tmpoutDf1 = outputParamDf_lat;
tmpoutDdf1 = outputParamDdf_lat;
%%
load('NN_test_CPGDay1_RandDataLat10kTrafo2.mat');
tmpinLat2 = inputParam_lat;
tmpoutDf2 = outputParamDf_lat;
tmpoutDdf2 = outputParamDdf_lat;
%%
load('CPG_day1_test2_trainingData.mat');
tmpinLat3 = inputParam_lat;
tmpoutDf3 = outputParamDf_lat;
tmpoutDdf3 = outputParamDdf_lat;
%%
load('CPG_day1_sim1_trainingData.mat');
tmpinLat4 = inputParam_lat;
tmpoutDf4 = outputParamDf_lat;
tmpoutDdf4 = outputParamDdf_lat;
%% 
load('CPG_day1_sim2_trainingData.mat');
tmpinLat5 = inputParam_lat;
tmpoutDf5 = outputParamDf_lat;
tmpoutDdf5 = outputParamDdf_lat;

%% Merge all data now 

inputParam_lat = [tmpinLat1;tmpinLat2;tmpinLat3;tmpinLat4;tmpinLat5];
outputParamDf_lat = [tmpoutDf1;tmpoutDf2;tmpoutDf3;tmpoutDf4;tmpoutDf5];
outputParamDdf_lat = [tmpoutDdf1; tmpoutDdf2; tmpoutDdf3; tmpoutDdf4; tmpoutDdf5];
%%
save('NN_test_CPGDay1_TotalDataLatTrafo2.mat','inputParam_lat','outputParamDf_lat','outputParamDdf_lat');
%%
