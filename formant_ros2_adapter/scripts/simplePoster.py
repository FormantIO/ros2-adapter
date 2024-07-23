#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import sys

# python3 simplePoster.py ./test_image.jpg
# python3 simplePoster.py ./test_video_600x300.mp4
def publish_message(media_file):

    is_video = False

    # Node is publishing to the video_frames topic using
    # the message type Image
    pubCompressed = rospy.Publisher(
        "video_frames_compressed", CompressedImage, queue_size=10
    )
    pubUncompressed = rospy.Publisher("video_frames", Image, queue_size=10)

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node("video_pub_py", anonymous=True)

    # Go through the loop 10 times per second
    rate = rospy.Rate(25)  # 10hz

    # if it is a video file
    if media_file.endswith(".mp4") or media_file.endswith(".mpv"):
        is_video = True
        cap = cv2.VideoCapture(media_file)

    print("ROS1 poster: {}".format(media_file))

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # While ROS is still running.
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        if is_video:
            ret, frame = cap.read()

            if ret is True:
                # Print debugging information to the terminal
                rospy.loginfo("publishing video frame %s", frame.shape)
                pubCompressed.publish(br.cv2_to_compressed_imgmsg(frame))
                pubUncompressed.publish(br.cv2_to_imgmsg(frame))
        else:
            print(" ROS1 -- publishing image --")
            img = cv2.imread(image, 0)
            pubCompressed.publish(br.cv2_to_compressed_imgmsg(img))
            pubUncompressed.publish(br.cv2_to_imgmsg(img))

        # Sleep just enough to maintain the desired rate
        rate.sleep()


if __name__ == "__main__":
    try:
        ## retrieve the video file from the command line
        file = rospy.myargv(argv=sys.argv)[1]
        publish_message(file)
    except rospy.ROSInterruptException:
        pass
