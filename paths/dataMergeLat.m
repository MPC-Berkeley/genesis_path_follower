%% File to merge laps data from RFS/CPG laps
clear all; close all; clc;

%% Load all the files to merge from 
load('CPG_day1_test1_trainingData.mat');
tmpinLat1 = inputParam_lat;
tmpoutDf1 = outputParamDf_lat;
tmpoutDdf1 = outputParamDdf_lat;
%%
% load('NN_test_CPGDay1_RandDataLat10kTrafo2.mat');
% tmpinLat2 = inputParam_lat;
% tmpoutDf2 = outputParamDf_lat;
% tmpoutDdf2 = outputParamDdf_lat;
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

inputParam_lat = [tmpinLat1;tmpinLat3;tmpinLat4;tmpinLat5];
outputParamDf_lat = [tmpoutDf1;tmpoutDf3;tmpoutDf4;tmpoutDf5];
outputParamDdf_lat = [tmpoutDdf1; tmpoutDdf3; tmpoutDdf4; tmpoutDdf5];
%%
save('NN_test_CPGDay1_SimTestTrajDataLat.mat','inputParam_lat','outputParamDf_lat','outputParamDdf_lat');
%%
vpred = inputParam_lat(:,4:11);
max(vpred,[],1)
min(vpred,[],1)

dvpred = vpred(:,2:end)-vpred(:,1:end-1);
max(dvpred,[],1)
min(dvpred,[],1)

%%
cpred = inputParam_lat(:,12:end);
max(cpred,[],1)
min(cpred,[],1)

dcpred = cpred(:,2:end) - cpred(:,1:end-1);
max(dcpred,[],1)
min(dcpred,[],1)

%%
vcpred = vpred .* cpred;
max(vcpred,[],1)
min(vcpred,[],1)

%%
dfprev = inputParam_lat(:,3);
max(dfprev)
min(dfprev)

%%
ey0 = inputParam_lat(:,1);
max(ey0)
min(ey0)
