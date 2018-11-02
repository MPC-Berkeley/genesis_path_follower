%% Script to prove TF Training is bad for lateral 
clear all; 
clc; close all;

%% 
load('NN_test_trainingData.mat'); load('trained_weightsPrimalLatTrajData.mat');
matin = inputParam_lat';
matout = outputParamDdf_lat'; 
diff_check = zeros(1,size(matin,2)); 
%% 
for i = 1:size(matin,2)
    l1 = max(0,W1*matin(:,i) + b1);
    l2 = max(0,W2*l1 + b2);
    l0 = W0*l2 + b0;
    
    diff_check(1,i) = norm(l0-matout(:,i),2);
end

%% Show stuff 
min_diff = min(diff_check)
max_diff = max(diff_check)
mean_diff = mean(diff_check)



