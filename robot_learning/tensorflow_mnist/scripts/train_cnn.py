#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tensorflow as tf
import cv2
import input_data
import numpy as np
mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)

def change_size(X):
	X1 = []
	for image in X:
	   image = image.reshape((28,28))
	   X1.append(image)
	X = np.asarray(X1)
	h = X.shape[1]
	w = X.shape[2]

	return X.reshape((X.shape[0], h, w, 1))


def weight_variable(shape):
  initial = tf.truncated_normal(shape, stddev=0.1)
  return tf.Variable(initial)

def bias_variable(shape):
  initial = tf.constant(0.1, shape=shape)
  return tf.Variable(initial)

def conv2d(x, W):
  return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], 
                      padding='SAME')

def max_pool_2x2(x):
  return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                        strides=[1, 2, 2, 1], padding='SAME')


def makeCNN(x,keep_prob):

    # --- define CNN model
    W_conv1 = weight_variable([5, 5, 1, 32])
    b_conv1 = bias_variable([32])
    h_conv1 = tf.nn.relu(conv2d(x, W_conv1) + b_conv1)

    h_pool1 = max_pool_2x2(h_conv1)

    W_conv2 = weight_variable([3, 3, 32, 64])
    b_conv2 = bias_variable([64])
    h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)

    h_pool2 = max_pool_2x2(h_conv2)

    W_fc1 = weight_variable([7 * 7 * 64, 1024])
    b_fc1 = bias_variable([1024])
    h_pool2_flat = tf.reshape(h_pool2, [-1, 7 * 7 * 64])
    h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)
    
    h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

    W_fc2 = weight_variable([1024, 10])
    b_fc2 = bias_variable([10])

    y_conv = tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)
 
    return y_conv

x = tf.placeholder(tf.float32, [None, 28, 28, 1])
y = tf.placeholder(tf.float32, [None, 10])

keep_prob = tf.placeholder(tf.float32)

y_conv = makeCNN(x,keep_prob)

cross_entropy = -tf.reduce_sum(y*tf.log(y_conv))
train_step = tf.train.AdamOptimizer(1e-4).minimize(cross_entropy)
correct_prediction = tf.equal(tf.argmax(y_conv,1), tf.argmax(y,1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))

saver = tf.train.Saver(keep_checkpoint_every_n_hours = 1.0)

sess = tf.InteractiveSession()
sess.run(tf.global_variables_initializer())

batchSize = 50

for i in range(20000):
  batch = mnist.train.next_batch(50)
  image = change_size(batch[0])
  if i%1000 == 0:
    train_accuracy = accuracy.eval(feed_dict={
        x:image, y: batch[1], keep_prob: 1.0})
    print("step %d, training accuracy %g"%(i, train_accuracy))
  train_step.run(feed_dict={x: image, y: batch[1], keep_prob: 0.5})
save_path = saver.save(sess, "model/model.ckpt")








