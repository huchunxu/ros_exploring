#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import cv2
import numpy as np
import input_data  
import tensorflow as tf

class MNIST():
    def __init__(self):
        image_topic = rospy.get_param("~image_topic", "")

        self._cv_bridge = CvBridge()

        #MNIST数据输入  
        self.mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)  
          
        self.x = tf.placeholder(tf.float32,[None, 784]) #图像输入向量  
        self.W = tf.Variable(tf.zeros([784,10]))        #权重，初始化值为全零  
        self.b = tf.Variable(tf.zeros([10]))            #偏置，初始化值为全零  
          
        #进行模型计算，y是预测，y_ 是实际  
        self.y = tf.nn.softmax(tf.matmul(self.x, self.W) + self.b)  
          
        self.y_ = tf.placeholder("float", [None,10])  
          
        #计算交叉熵  
        self.cross_entropy = -tf.reduce_sum( self.y_*tf.log(self.y))  
        #接下来使用BP算法来进行微调,以0.01的学习速率  
        self.train_step = tf.train.GradientDescentOptimizer(0.01).minimize(self.cross_entropy)  
          
        #上面设置好了模型，添加初始化创建变量的操作  
        self.init = tf.global_variables_initializer()  
        #启动创建的模型，并初始化变量  
        self.sess = tf.Session()  
        self.sess.run(self.init)  

        #开始训练模型，循环训练1000次  
        for i in range(1000):  
            #随机抓取训练数据中的100个批处理数据点  
            batch_xs, batch_ys = self.mnist.train.next_batch(100)  
            self.sess.run(self.train_step, feed_dict={self.x:batch_xs, self.y_:batch_ys})  

        ''''' 进行模型评估 '''  
        #判断预测标签和实际标签是否匹配  
        correct_prediction = tf.equal(tf.argmax(self.y,1),tf.argmax(self.y_,1))   
        self.accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))  
       
        #计算所学习到的模型在测试数据集上面的正确率  
        print( "The predict accuracy with test data set: \n")
        print( self.sess.run(self.accuracy, feed_dict={self.x:self.mnist.test.images, self.y_:self.mnist.test.labels}) )  

        self._sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', Int16, queue_size=1)

    def callback(self, image_msg):
        #预处理接收到的图像数据
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        ret,cv_image_binary = cv2.threshold(cv_image_gray,128,255,cv2.THRESH_BINARY_INV)
        cv_image_28 = cv2.resize(cv_image_binary,(28,28))
        
        #转换输入数据shape,以便于用于网络中
        np_image = np.reshape(cv_image_28, (1, 784))

        predict_num = self.sess.run(self.y, feed_dict={self.x:np_image, self.y_:self.mnist.test.labels})
        
        #找到概率最大值
        answer = np.argmax(predict_num, 1)
        
        #发布识别结果
        rospy.loginfo('%d' % answer)
        self._pub.publish(answer)
        #rospy.sleep(1) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_tensorflow_mnist')
    tensor = MNIST()
    rospy.loginfo("ros_tensorflow_mnist has started.")
    tensor.main()
