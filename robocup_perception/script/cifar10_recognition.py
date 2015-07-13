#!/usr/bin/env python
#-*- coding: utf-8 -*-

# import ROS
import rospy
import rospkg
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from robocup_perception.srv import Classify
from robocup_perception.srv import ClassifyResponse

#import Caffe
import caffe
from caffe.proto import caffe_pb2
import numpy as np

def classfier(req):
    mean_blob= caffe_pb2.BlobProto()
    rospack = rospkg.RosPack()
    dir_path = rospack.get_path("robocup_perception")
    file_path = dir_path + '/recog_file'
    recog_file_path = dir_path + '/object_image'
    with open(file_path + '/mean.binaryproto') as f:
        mean_blob.ParseFromString(f.read())
        mean_array = np.array(
                mean_blob.data,
                dtype = np.float32
                ).reshape((
                    mean_blob.channels,
                    mean_blob.height,
                    mean_blob.width))
        recognition = caffe.Classifier(
        file_path + '/cifar10_quick.prototxt',
        file_path + '/cifar10_quick_iter_4000.caffemodel',
        mean = mean_array,
        raw_scale = 255)

        if req.flag == True:
            image_file_path = recog_file_path + '/' + req.image_name
            image = caffe.io.load_image(image_file_path + '.jpg')
            predictions = recognition.predict([image], oversample = False)
            pred=np.argmax(predictions)
            class_number = pred
            return ClassifyResponse(result_num = class_number)
        else:
            print "Not classify flag ."


def service_server():
    rospy.init_node("classify_server")
    s = rospy.Service('classify_call',Classify,classfier)
    rospy.spin()

if __name__ == '__main__':
    service_server()
