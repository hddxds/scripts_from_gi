import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():

    parser = argparse.ArgumentParser(description="Rosbag to images")
    parser.add_argument("bag", help="bag name")
    parser.add_argument("output_path", help="image path")
    parser.add_argument("topic", help="image topic")
    parser.add_argument("format", help="image format", const="png")

    args = parser.parse_args()

    print("input bag: ", args.bag)
    print("output image path: ", args.output_path)
    print("using image topic: ", args.topic)
    print("using image format: ", args.format)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()

    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg)

        cv2.imwrite(os.path.join(args.output_dir, "image%i." % count, args.format), cv_img)

    bag.close()


if __name__ == '__main__':
    main()
