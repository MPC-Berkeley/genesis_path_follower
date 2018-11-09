# Python Code that uses Tensorflow to Train the NNs for Lateral Controller 
# Monimoy Bujarbaruah: 10/30/2018
# No Rights Reserved

import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
import scipy.io as sio
from sklearn.utils import shuffle
from sklearn.preprocessing import normalize
import IPython
from sklearn.preprocessing import StandardScaler
from sklearn import preprocessing
import math

def normalize(x, mean, std, eps=1e-8):
    return (x - mean) / (std + eps)

def unnormalize(x, mean, std):
    return x * std + mean

#%% We have imported all dependencies
df = sio.loadmat('NN_test_CPGDay5_BadRandTrainingDataLat10k.mat',squeeze_me=True, struct_as_record=False) # read data set using pandas
# df = sio.loadmat('NN_test_trainingDataLatRFS.mat',squeeze_me=True, struct_as_record=False) # read data set using pandas
x_data = df['inputParam_lat']
y_data = df['outputParamDdf_lat']
x_data_t = x_data
y_data_t = y_data

###### TRY REMOVING #######
# x_data, y_data, y_dataDual = shuffle(x_data, y_data, y_dataDual)
x_data, y_data = shuffle(x_data, y_data)

#f = h5py.File('NN_test_trainingDataLat10k_PrimalDual2.mat')
#x_data = np.array(f['inputParam_lat'])
#y_data =  np.array(f['outputParamDdf_lat'])
#y_dataDual =  np.array(f['outputParamDual_lat'])

data_length = x_data.shape[0]
train_length = int(np.floor(data_length*4/5))           # 80% data to train 

## Split Training and Testing Data to not Overfit 
x_train = x_data[:train_length, :]                      # Training data
y_train = y_data[:train_length,:]                       # Training data
# y_trainDual = y_dataDual[:train_length,:]               # Training data 

x_test = x_data[train_length:, :]                       # Testing data
y_test = y_data[train_length:,:]   
y_data_t_test = y_data_t[train_length:, :]                     # Testing data
# y_testDual = y_dataDual[train_length:,:]              # Testing data 

insize =  x_data.shape[1]
outsize = y_data.shape[1]                               # Dimension of primal output data 
# outsizeD = y_dataDual.shape[1]                        # Dimension of dual output data 

xs = tf.placeholder(tf.float32, [insize, None])#tf.placeholder("float")
ys = tf.placeholder(tf.float32, [outsize, None])#tf.placeholder("float")
lr = tf.placeholder(tf.float32)

# ysD = tf.placeholder("float")
#%%

################## PRIMAL NN TRAINING ##############################
neuron_size = 20
neuron_sizeML = neuron_size                             # Can vary size of the intermediate layer as well

W_1 = tf.Variable(tf.random_uniform([neuron_size,insize]))
b_1 = tf.Variable(tf.random_uniform([neuron_size,1]))
layer_1 = tf.add(tf.matmul(W_1,xs), b_1)
layer_1 = tf.nn.relu(layer_1)

# layer 1 multiplying and adding bias then activation function
W_2 = tf.Variable(tf.random_uniform([neuron_sizeML,neuron_size]))
b_2 = tf.Variable(tf.random_uniform([neuron_sizeML,1]))
layer_2 = tf.add(tf.matmul(W_2,layer_1), b_2)
layer_2 = tf.nn.relu(layer_2)

# layer 2 multiplying and adding bias then activation function
W_O = tf.Variable(tf.random_uniform([outsize,neuron_sizeML]))
b_O = tf.Variable(tf.random_uniform([outsize,1]))
output = tf.add(tf.matmul(W_O,layer_2), b_O)

#  O/p layer multiplying and adding bias then activation function
#  notice output layer has one node only since performing #regression


########### DOES THIS COMPUTE NORM OR ELEMENTWISE SQUARE???? #############  
# https://stackoverflow.com/questions/41338509/tensorflow-mean-squared-error-loss-function
# loss = tf.reduce_sum(tf.pow(prediction - Y,2))/(n_instances)
# loss = tf.reduce_mean(tf.squared_difference(prediction, Y))
# loss = tf.nn.l2_loss(prediction - Y)   # does not normalize w.r.t. #samples

# WHAT IF WE USE A ONE-NORM INSTEAD OF 2-NORM?
# loss = tf.reduce_mean(tf.abs(y - y_data)) 
# optimizer = tf.train.GradientDescentOptimizer(0.05) TO REDUCE OSCILLATION?

# cost = tf.reduce_mean(tf.square(output-ys))            # our mean squared error cost function

cost = tf.losses.mean_squared_error(output, ys)          # tf.reduce_mean(tf.squared_difference(output, ys))#+ tf.abs(output - ys) )
train = tf.train.AdamOptimizer(lr).minimize(cost)        # GD and proximal GD working bad! Adam and RMS well.

c_t = []
c_test = []

##%%
with tf.Session() as sess:
     # Initiate session and initialize all vaiables
     sess.run(tf.global_variables_initializer())
     saver = tf.train.Saver()

     inds = np.arange(x_train.shape[0])
     train_count = len(x_train)

     N_EPOCHS = 150
     BATCH_SIZE = 32
     max_learning_rate = 0.001
     min_learning_rate = 0.0001
     #learning_rate = 0.0001
     decay_speed = 1200000.0
     lr_it = 0
     for i in range(0, N_EPOCHS):        
         
         for start, end in zip(range(0, train_count, BATCH_SIZE),
                               range(BATCH_SIZE, train_count + 1,BATCH_SIZE)):
             learning_rate = min_learning_rate + (max_learning_rate - min_learning_rate) * math.exp(-lr_it/decay_speed)
             lr_it += 1
             sess.run([cost,train], feed_dict={xs: np.transpose(x_train[start:end]),
                                            ys: np.transpose(y_train[start:end]), lr: learning_rate})
    
         c_t.append(sess.run(cost, feed_dict={xs:np.transpose(x_train),ys:np.transpose(y_train)}))
         c_test.append(sess.run(cost, feed_dict={xs:np.transpose(x_test),ys:np.transpose(y_test)}))
         print('Epoch :',i,'Cost Train :',c_t[i], 'Cost Test :',c_test[i], 'learning rate',learning_rate)

#%% Saving weight matrices 
     vj={}
     vj['W1'] = sess.run(W_1)
     vj['W2'] = sess.run(W_2)
     vj['W0'] = sess.run(W_O)
     vj['b1'] = sess.run(b_1)
     vj['b2'] = sess.run(b_2)
     vj['b0'] = sess.run(b_O)
     sio.savemat('trained_weightsPrimalLatBadVCRand10kData_CPGDay5.mat',vj)



################################ Plotting the Primal NN Train Quality
# plt.plot(range(len(c_t)),c_t, 'r')
# plt.ylabel('Error in Training')
# plt.xlabel('Epoch')
# plt.title('Fitting Error Training')
# plt.show()
# # plt.hold(True)                                          
# plt.plot(range(len(c_test)),c_test, 'b')
# plt.ylabel('Error in Testing Points')
# plt.xlabel('Epoch')
# plt.title('Fitting Testing Error')
# plt.show()