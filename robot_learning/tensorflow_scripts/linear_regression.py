#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tensorflow as tf
import numpy as np

# 从-1到1等差采样101个点
trX = np.linspace(-1, 1, 101)
# 根据线性关系计算每个x对应的y值，并且加入一些噪声
trY = 2 * trX + \
	np.ones(*trX.shape) * 4 + \
	np.random.randn(*trX.shape) * 0.03

# 创建变量占位符
X = tf.placeholder(tf.float32)
Y = tf.placeholder(tf.float32)

# 创建模型
w = tf.Variable(0.0, name="weights")
b = tf.Variable(0.0, name="biases")
y_model = tf.multiply(X, w) + b  # 线性回归模型y=X*w + b

# 创建代价函数
cost = tf.square(Y - y_model) 

# 构建一个优化器，尽量拟合所有的数据点，使得代价函数的值最小
train_op = tf.train.GradientDescentOptimizer(0.01).minimize(cost)

# 在会话中运行计算图
sess = tf.Session()
# 初始化所有变量
init = tf.global_variables_initializer()
sess.run(init)

# 开始训练模型
for i in range(100):
	for (x, y) in zip(trX, trY):
		sess.run(train_op, feed_dict={X: x, Y: y})
w_ = sess.run(w) # it should be something around 2
b_ = sess.run(b) # it should be something atound 4

# 打印训练得到的线性回归模型
print("Result : trY = " + str(w_) + "*trX + " + str(b_))
