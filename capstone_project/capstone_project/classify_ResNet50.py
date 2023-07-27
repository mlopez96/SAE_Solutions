#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
import tensorflow as tf
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from keras.preprocessing import image
from keras.applications.resnet import ResNet50, preprocess_input, decode_predictions
from predictor_msgs.msg import Predictor

GPU_OPTIONS = tf.compat.v1.GPUOptions(allow_growth=True)
CONFIG = tf.compat.v1.ConfigProto(gpu_options=GPU_OPTIONS)
CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.5

sess = tf.compat.v1.Session(config=CONFIG)
tf.compat.v1.keras.backend.set_session(sess)
      
class ResNet50_Classifier(Node):
    def __init__(self):
        super().__init__("ResNet50_Classifier")
        self.image_sub = self.create_subscription(Image, '/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Predictor, '/object_detector', 10)
        self.bridge_object = CvBridge()
        self.target_size = (224, 224)
        self.sess = sess
        self.model = ResNet50(weights='imagenet')
        self.graph = tf.compat.v1.get_default_graph()
             

    def callback(self, image_msg):
      #First convert the image to OpenCV image 
      
      
      cv_image = self.bridge_object.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
      cv_image = cv2.resize(cv_image, self.target_size) # resize image
      height, width, channels = cv_image.shape
      
      np_image = np.asarray(cv_image) # read as np array
      np_image = np.expand_dims(np_image, axis=0) # Add another dimension for tensorflow
      np_image = np_image.astype(float) # preprocess needs float64 and img is uint8

      np_image = preprocess_input(np_image) # Regularize the data
      preds = self.model.predict(np_image) # Classify the image
      pred_string = decode_predictions(preds, top=1) # Decode top 1 predictions
      
      pub_msg = Predictor()
      pub_msg.header.stamp = self.get_clock().now().to_msg()
      pub_msg.label = pred_string[0][0][1]
      pub_msg.score = int(pred_string[0][0][2])
      pub_msg.box_coords = []
      self.pub.publish(pub_msg)
      
def main(args=None):
    print("Object Recognition Started")

    rclpy.init(args=args)
    node = ResNet50_Classifier()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
      


