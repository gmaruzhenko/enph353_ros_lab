#!/usr/bin/env python
import cv2
import numpy as np
import time

import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


Kernel_size = 15
low_threshold = 40
high_threshold = 80


class image_converter:

    def __init__(self):
        self.error_pub = rospy.Publisher("/rrbot/cv_stear_error", Int16, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/rrbot/camera1/image_raw", Image, self.callback)

    def callback(self, data):
        rate = rospy.Rate(10) # 10hz
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(0)

        #process_image(cv_image)
        

        while not rospy.is_shutdown():
            hello_str = 15
            rospy.loginfo(hello_str)
            self.error_pub.publish(hello_str)
            rate.sleep()

        # try:
        #     self.error.publish(self.bridge.cv2_to_imgmsg(1))
        # except CvBridgeError as e:
        #     print(e)


def process_image(image):
    # frame count itterator setup
    count = 0
    success = True
    while success:
        # Convert to Grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Blur image to reduce noise. if Kernel_size is bigger the image will be more blurry
        blurred = cv2.GaussianBlur(gray, (Kernel_size, Kernel_size), 0)

        # debug to find size of image
        # print blurred.shape

        # crop down to last few slices (actually not needed)
        crop_img = blurred[200:240, 0:320]

        # cv2.imshow("cropped", crop_img)
        # cv2.waitKey(0)

        # Perform canny edge-detection.
        # If a pixel gradient is higher than high_threshold is considered as an edge.
        # if a pixel gradient is lower than low_threshold is is rejected , it is not an edge.
        # Bigger high_threshold values will provoque to find less edges.
        # Canny recommended ratio upper:lower  between 2:1 or 3:1
        edged = cv2.Canny(crop_img, low_threshold, high_threshold)

        # cv2.imshow("cropped", edged)
        # cv2.waitKey(0)

        # Note Black is 0 white is 1
        # Find first and second occurance of contour plot on second to
        # last row of pixel (approximation of real path)
        first_white = 0
        second_white = 0

        # due to image size
        index_max = 320
        index = 0

        while index < index_max:

            if edged[39, index] > 0:
                if first_white == 0:
                    first_white = index
                else:
                    second_white = index

            index += 1

        average_white = int((first_white+second_white)/2)

        # print average_white

        # Now draw circle showing the center location of our contour line
        cv2.circle(image, (average_white, 210), 20, (0, 0, 255), 1)

        # run the slideshow at a min wait of 1 ms
        cv2.imshow("cropped", image)
        cv2.waitKey(0)

        # reset image for next frame

        count += 1


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
