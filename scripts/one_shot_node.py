#!/usr/bin/env python2.7

from sys import argv
import rospy
import rospkg
from sift_gpu.srv import SIFTMatchRequest, SIFTMatch, SIFTMatchResponse
from cv_bridge import CvBridge
import cv2

if __name__ == "__main__":
    rospy.init_node("one_shot")
    sift_client = rospy.ServiceProxy('/sift_gpu/sift_match', SIFTMatch)
    sift_client.wait_for_service()

    cv_bridge = CvBridge()
    rospack = rospkg.RosPack()
    img = cv2.imread(rospack.get_path("sift_gpu") + '/testset/box_in_scene.png')
    img_msg = cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
    req = SIFTMatchRequest()
    req.query_img = img_msg
    resp = sift_client(req)
    rospy.loginfo(resp)