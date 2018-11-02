%% Training with Genesis data 
% Modified by Monimoy Bujarbaruah 
clc; clear all; close all; 
%% Load data from CVXGen output file
load('NN_test_trainingDataLong10k_PrimalDual.mat');
x_nn_train = inputParam_long'; 
y_nn_train = outputParamDacc_long'; 
y_nn_train_dual = outputParamDual_long';
%% Now Neural Net Train for Genesis Control with all above data 
% https://www.mathworks.com/help/releases/R2016b/nnet/ref/network.html

maxTrials_fixedStructure = 5;                  % max number of fitting trials for given networkSize
maxTrials = 5;                                % total number of trials of neuron increase                 (keep same for all networks) 
net = cell(1,1);                               % Just one NN defined here 
%% 
trialNum_AddNeur = 1; 

% store all runs
net_all = cell(maxTrials_fixedStructure,maxTrials);
net_perf_all = nan(maxTrials_fixedStructure,maxTrials);

% init network parameters 
tr_mse = inf;                                                                   % training error; need both not to overfit
networkSize = [10 10];

tmp = sqrt(sum(y_nn_train.^2,1));                                               % length of indiv training data
eps_mse = 1e-6*max(tmp);                                                        % desired MSE error
%% 
while (tr_mse > eps_mse) && (trialNum_AddNeur <= maxTrials)

    networkSize = networkSize + 5;                                  % if passed vector, then multiple layers
    disp(['network size: ' , num2str(networkSize)])

    trialNum_FixedStruc = 0;

    for ii = 1 : maxTrials_fixedStructure                                                                       % different training give in different results...

        trialNum_FixedStruc = trialNum_FixedStruc+1;                                                    % increase trial number
        disp(['trial number with fixed structure: ' num2str(trialNum_FixedStruc)])


        %% Define network
        net = feedforwardnet(networkSize);                                                  % 2 good enough for u0
    
        net.layers{1}.transferFcn = 'poslin';                                                   % change to sigmoid (Rel-U Bad Now) 
        net.layers{2}.transferFcn = 'poslin';     

%         net.layers{1}.transferFcn = 'tansig';                                                   % change to sigmoid (Rel-U Bad Now) 
%         net.layers{2}.transferFcn = 'tansig';     
      % Different sets are randomly created for training, validation and testing the network

        net.divideParam.trainRatio =  80/100;
        net.divideParam.valRatio   =   0/100;
        net.divideParam.testRatio  =   20/100;
        net.trainParam.epochs      =   100;     
        net.output.processFcns = {'mapminmax'};
        net.trainParam.epochs = 18500; 
     
        
        net = configure(net,x_nn_train,y_nn_train);                             % configure network (#inputs/output, params, ...)
  
 [net, tr] = train(net,x_nn_train,y_nn_train,'useGPU','no','useParallel','yes');
 
 tr_test_mse = tr.best_tperf;                                                   % There is epoch 0, but arrays in MATLAB start in 1.
 tr_mse      = tr.best_perf; 
  
 net_perf_all(trialNum_FixedStruc,trialNum_AddNeur) = tr_test_mse;
 net_all{trialNum_FixedStruc, trialNum_AddNeur} = net;


        % mse_val = tr.vperf(tr.best_epoch + 1);
        % mse_test = tr.tperf(tr.best_epoch + 1);
        disp(['Best testing error during training: ', num2str(tr_test_mse) ])
        disp(['best testing error so far during training: ', num2str(min(min(net_perf_all)))])
        if tr_mse <= eps_mse && tr_test_mse <=eps_mse
            disp(['---> Success!   |   NetworkSize: ',  num2str(networkSize) , '   |   MSE: ', num2str(tr_mse) ])
            break
        end
    end

    trialNum_AddNeur = trialNum_AddNeur + 1; 
    disp(['Neuron Increase Index: ' num2str(trialNum_AddNeur)])

end

%%
% keyboard
    
    
%% Picking Best Trained Network
min_m = min(min(net_perf_all));
[idx1, idx2] = find(net_perf_all==min_m);
net = net_all{idx1,idx2};                                           % Best Network after training 

perf = net_perf_all(idx1,idx2);                                    % Best Network after training 

%% Train Dual Network 
x_nn_train_dual = x_nn_train; 
%%

maxTrials_fixedStructureD = 5;          % max number of fitting trials for given networkSize
maxTrialsD = 4;                               % total number of trials with neuron increase 

trialNum_AddNeur = 1; 
net_dual_all = cell(maxTrials_fixedStructureD,maxTrialsD);
net_dual_perf_all = nan(maxTrials_fixedStructureD,maxTrialsD);

% init network parameters 
tr_mse = inf;                           % training error; stopping error
networkSize = [15 15];

% tmp = sqrt(sum(lambda_reg_all.^2,1));   % length of indiv training data
tmp = sqrt(sum(y_nn_train_dual.^2,1));             % length of indiv training data
eps_mse = 1e-18*max(tmp);

%%
while (tr_mse > eps_mse) && (trialNum_AddNeur < maxTrialsD)
    
    networkSize = networkSize + 5;                      % if passed vector, then multiple layers
    disp(['network size: ' , num2str(networkSize)])

    trialNum_FixedStruc = 0;
        
    
    for ii = 1 : maxTrials_fixedStructureD                     % different training give in different results...
        clear net_dual y_nn_trained perf x_nn_new y_nn_new     % clear previous data

        trialNum_FixedStruc = trialNum_FixedStruc+1;                                % increase trial number
        disp(['trial number with fixed structure: ' num2str(trialNum_FixedStruc)])

        % define network
        net_dual = feedforwardnet(networkSize);          % 2 good enough for u0
        
%       net_dual.layers{1}.transferFcn = 'tansig';      % change to rectified linear units (BEST)
%       net_dual.layers{2}.transferFcn = 'tansig';      % change to rectified linear units (BEST)

        net_dual.layers{2}.transferFcn = 'poslin';     
        net_dual.layers{1}.transferFcn = 'poslin';

        % change to rectified linear units (BEST)
        

        % Different sets are randomly created for training, validation and testing the network
        net_dual.divideParam.trainRatio = 80/100;
        net_dual.divideParam.valRatio = 0/100;
        net_dual.divideParam.testRatio = 20/100;
        net_dual.output.processFcns = {'mapminmax'};
        net_dual.trainParam.epochs = 18500; 
        
        % view(net)
        net_dual = configure(net_dual, x_nn_train_dual, y_nn_train_dual); % configure network (#inputs/output, params, ...)
        % view(net)    
        [net_dual tr_dual] = train(net_dual, x_nn_train_dual, y_nn_train_dual,'useGPU','no','useParallel','yes');                  % train network

        % Once the network has been trained, we can obtain the Mean Squared Error
        % for the best epoch (time when the training has stopped in order to avoid
        % overfitting the network).
        tr_mse = tr_dual.perf(tr_dual.best_epoch + 1); % There is epoch 0, but arrays in MATLAB start in 1.
        net_dual_perf_all(trialNum_FixedStruc, trialNum_AddNeur) = tr_mse;
        net_dual_all{trialNum_FixedStruc, trialNum_AddNeur} = net_dual;
        
        % mse_val = tr.vperf(tr.best_epoch + 1);
        % mse_test = tr.tperf(tr.best_epoch + 1);
        disp(['network size: ' , num2str(networkSize)])
        disp(['training error: ', num2str(tr_mse) ])
        disp(['best training error so far: ', num2str(min(min(net_dual_perf_all)))])
        if tr_mse <= eps_mse
            disp(['---> Dual Success!   |   NetworkSize: ',  num2str(networkSize) , '   |   MSE: ', num2str(tr_mse) ])
            break
        end
    end
    
    trialNum_AddNeur = trialNum_AddNeur + 1; 
    disp(['Neuron Increase Index: ' num2str(trialNum_AddNeur)])

end

%%
min_mD = min(min(net_dual_perf_all));
[idx1, idx2] = find(net_dual_perf_all== min_mD);
net_dual = net_dual_all{idx1,idx2};
perf_dual = net_dual_perf_all(idx1,idx2)

% keyboard

%% Extract the weights and biases of the networks
Wi_PLong = net.Iw{1};
bi_PLong = net.b{1};
W1_PLong = net.Lw{2};
b1_PLong = net.b{2};
Wout_PLong = net.Lw{6};
bout_PLong = net.b{3}; 

%% 
Wi_DLong = net_dual.Iw{1};
bi_DLong = net_dual.b{1};
W1_DLong = net_dual.Lw{2};
b1_DLong = net_dual.b{2};
Wout_DLong = net_dual.Lw{6};
bout_DLong = net_dual.b{3}; 
%% Saving trained nets
save('trained_netPrimVxParam2to25_SF10Fy1_tightenedInputs1000_3e4Data_ReLU.mat','net_all','net_perf_all'); 
%% How to make the functions 
%  Run after loading all trained nets from trained_nets.mat
%  genFunction(net{number},'netSimFunc_name','matrixonly',yes');

