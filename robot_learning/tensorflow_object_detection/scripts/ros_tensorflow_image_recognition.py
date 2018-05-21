#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import classify_image

class ImageRecognition():
    def __init__(self):
        image_topic = rospy.get_param("~image_topic", "")

        classify_image.maybe_download_and_extract()
        self._session = tf.Session()
        classify_image.create_graph()
        self._cv_bridge = CvBridge()

        self._sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', String, queue_size=1)
        self.score_threshold = rospy.get_param('~score_threshold', 0.1)
        self.use_top_k = rospy.get_param('~use_top_k', 5)

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # copy from
        # classify_image.py
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()
        # Creates graph from saved GraphDef.
        softmax_tensor = self._session.graph.get_tensor_by_name('softmax:0')
        predictions = self._session.run(
            softmax_tensor, {'DecodeJpeg/contents:0': image_data})
        predictions = np.squeeze(predictions)
        # Creates node ID --> English string lookup.
        node_lookup = classify_image.NodeLookup()
        top_k = predictions.argsort()[-self.use_top_k:][::-1]
        for node_id in top_k:
            human_string = node_lookup.id_to_string(node_id)
            score = predictions[node_id]
            if score > self.score_threshold:
                rospy.loginfo('%s (score = %.5f)' % (human_string, score))
                self._pub.publish(human_string)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    classify_image.setup_args()
    rospy.init_node('ros_tensorflow_image_recognition')
    tensor = ImageRecognition()
    tensor.main()
