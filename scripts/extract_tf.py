import rosbag
import rospy
import yaml
from argparse import ArgumentParser
import os

parser = ArgumentParser()

parser.add_argument("bag_file",
                    help="Name of bag file to extract transforms from.")

parser.add_argument("-f", "--frame_id", dest="frame_id", required=True,
                    help="Frame id of transforms to extract.")

parser.add_argument("-c", "--child_frame_id", dest="child_frame_id", required=True,
                    help="Child frame id of transforms to extract.")

parser.add_argument("-t", "--tf_topic", dest="tf_topic", default="/tf",
                    help="Topic to read transforms from.")

args = parser.parse_args()

bag = rosbag.Bag(args.bag_file)

encoder = open(args.bag_file[:-4] + "_tf.txt", 'w')
encoder.write("%time tx ty tz qw qx qy qz\n")

for topic, msg, t in bag.read_messages(topics=[args.tf_topic]):
    for transform in msg.transforms:
        if (transform.header.frame_id == args.frame_id and transform.child_frame_id == args.child_frame_id):
            encoder.write(str(transform.header.stamp.to_sec()))
            encoder.write(" ")
            encoder.write(str(transform.transform.translation.x))
            encoder.write(" ")
            encoder.write(str(transform.transform.translation.y))
            encoder.write(" ")
            encoder.write(str(transform.transform.translation.z))
            encoder.write(" ")
            encoder.write(str(transform.transform.rotation.w))
            encoder.write(" ")
            encoder.write(str(transform.transform.rotation.x))
            encoder.write(" ")
            encoder.write(str(transform.transform.rotation.y))
            encoder.write(" ")
            encoder.write(str(transform.transform.rotation.z))
            encoder.write("\n")

bag.close()

exit(0)
