#!/usr/bin/env python

# !/usr/bin/env python

import rospy
import tf
from ros_object_detection.msg import BoundingBoxes
from fiducial_msgs.msg import FiducialTransformArray

class Callbacks:
    def __init__(self):
        self.boxes = None

        rospy.Subscriber("/bounding_box", BoundingBoxes, self.box_callback)


    def box_callback(self, data):
        self.boxes = data
        for i, box in enumerate(self.boxes.data):

            br = tf.TransformBroadcaster()
            parent = rospy.get_param('parent', 'map')
            cl = box.data
            idx = i
            x_0 = ( (box.xmin*2)-1 + (box.xmax*2)-1 )/ 2
            y_0 = ( -((box.ymin*2)-1) + -((box.ymax*2)-1) )/ 2
            br.sendTransform([x_0, y_0, 0], [0, 0, 0, 1], rospy.Time.now(), '{}_{}'.format(cl, idx), parent)
            pass


if __name__ == '__main__':
    rospy.init_node('show_tfs')
    callbacks = Callbacks()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
