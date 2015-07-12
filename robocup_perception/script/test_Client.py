#!/usr/bin/env python
import rospy
from robocup_perception.srv import Classify
import sys

if __name__ == '__main__':
    rospy.init_node('test_classify')
    classify_call = rospy.ServiceProxy('classify_call',Classify)
    name = str(sys.argv[1])
    flag = True
    response = classify_call(name, flag)
    
    print(response.result_num)

