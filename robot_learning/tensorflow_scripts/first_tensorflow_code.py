#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 包含tensorflow库
import tensorflow as tf

# 定义两个矩阵
matrix1 = tf.constant([[1., 2.],[3., 4.]])
matrix2 = tf.constant([[4., 3.],[2., 1.]])

# 定义一个字符串
message = tf.constant('Results of matrix operations')

# 矩阵相乘
product = tf.matmul(matrix1, matrix2)

# 定义一个会话
sess = tf.Session()
# 在会话中运行以上运算
result = sess.run(product)

# 输出计算结果
print(sess.run(message))
print(result)

# 关闭会话
sess.close()
