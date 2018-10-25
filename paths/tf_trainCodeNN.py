import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
import pdb
import scipy.io as sio

# We have imported all dependencies
df = np.genfromtxt('Real_ACC_YJ.csv', delimiter=',') # read data set using pandas
x_data = df[:, :2]
y_data = df[:, 2:]

data_length = x_data.shape[0]

x_train = x_data[:data_length, :]    # Training and testing on the same data
x_test =  x_data[:data_length, :]

y_train = y_data[:data_length,:]      # Training and testing on the same data. Over Entire Horizon now
y_test =  y_data[:data_length,:]

outsize = y_data.shape[1]

# Extra Evaluation Points 
df2 = np.genfromtxt('new_eval_points.csv', delimiter=',') # read data set using pandas 
x_test2 = df2
    
xs = tf.placeholder("float")
ys = tf.placeholder("float")

neuron_size = 50 
neuron_sizeML = neuron_size    # Can vary size of the intermediate layer as well

W_1 = tf.Variable(tf.random_uniform([2,neuron_size]))
b_1 = tf.Variable(tf.random_uniform([neuron_size]))
layer_1 = tf.add(tf.matmul(xs,W_1), b_1)
layer_1 = tf.nn.relu(layer_1)
    
# layer 1 multiplying and adding bias then activation function
W_2 = tf.Variable(tf.random_uniform([neuron_size,neuron_sizeML]))
b_2 = tf.Variable(tf.random_uniform([neuron_sizeML]))
layer_2 = tf.add(tf.matmul(layer_1,W_2), b_2)
layer_2 = tf.nn.relu(layer_2)


# layer 2 multiplying and adding bias then activation function
W_O = tf.Variable(tf.random_uniform([neuron_sizeML,outsize]))
b_O = tf.Variable(tf.random_uniform([outsize]))
output = tf.add(tf.matmul(layer_2,W_O), b_O)

# O/p layer multiplying and adding bias then activation function
# notice output layer has one node only since performing #regression

cost = tf.reduce_mean(tf.square(output-ys))
# our mean squared error cost function
train = tf.train.AdamOptimizer(0.0001).minimize(cost)  # GD and proximal GD working bad! Adam and RMS well. 
# Gradinent Descent optimiztion just discussed above for updating weights and biases

c_t = []
c_test = []


with tf.Session() as sess:
    # Initiate session and initialize all vaiables
    sess.run(tf.global_variables_initializer())
    saver = tf.train.Saver()

    inds = np.arange(x_train.shape[0]) 
    train_count = len(x_train)  
    
    N_EPOCHS = 10000
    BATCH_SIZE = 32

  

    for i in range(1, N_EPOCHS + 1):
        for start, end in zip(range(0, train_count, BATCH_SIZE),
                              range(BATCH_SIZE, train_count + 1,BATCH_SIZE)):

            sess.run([cost,train], feed_dict={xs: x_train[start:end],
                                           ys: y_train[start:end]})    
#        for j in inds:
#            sess.run([cost,train],feed_dict= {xs:np.reshape(x_train[j,:], (1, 2)), ys:y_train[j]})
            # Run cost and train with each sample
        c_t.append(sess.run(cost, feed_dict={xs:x_train,ys:y_train}))
#       print('Epoch :',i,'Cost :',c_t[i])
    pred = sess.run(output, feed_dict={xs:x_test})
    pred_new_points = sess.run(output, feed_dict={xs:x_test2})
    # predict output of test data after training
    print('Cost :',sess.run([output, cost], feed_dict={xs:x_test,ys:y_test}))
    
    
#  Getting the variables from the Neural Net 
    vj={}
    vj['W1'] = sess.run(W_1)
    vj['W2'] = sess.run(W_2)
    vj['W0'] = sess.run(W_O)
    vj['b1'] = sess.run(b_1)
    vj['b2'] = sess.run(b_2)
    vj['b0'] = sess.run(b_O)
    
    test_stuff = {}
    test_stuff['pred_out_orgdata'] = pred
    test_stuff['pred_out_newdata'] = pred_new_points
    test_stuff['label_test_orgdata'] = y_test
    
    sio.savemat('test_qual_check.mat',test_stuff)
    sio.savemat('trained_weights.mat',vj)
    
#    pdb.set_trace()
    
   # Denormalize data     
    plt.plot(range(len(c_t)),c_t, 'r')
    plt.ylabel('Error in Training')
    plt.xlabel('Epoch')
    plt.title('Fitting Error Training')
    plt.show()
    
    
#    if input('Save model ? [Y/N]') == 'Y':
#        saver.save(sess,'yahoo_dataset.ckpt')
#        print('Model Saved')
        
        
