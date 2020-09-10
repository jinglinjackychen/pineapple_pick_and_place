#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def cb(msg):
	cam_info = CameraInfo()
	cam_info.header = msg.header
	cam_info.height = msg.height
	cam_info.width = msg.width
	cam_info.distortion_model = msg.distortion_model
	cam_info.D = [0.15694425605041176, -0.21189615914006485, -0.013390544706847574, 0.01046160820450328, 0.0]
	cam_info.K = [626.6125988031955, 0.0, 327.8649703938749, 0.0, 624.2311253966595, 217.84135969608295, 0.0, 0.0, 1.0]
	cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	cam_info.P = [649.4646606445312, 0.0, 333.27927896346955, 0.0, 0.0, 646.218017578125, 212.56643737078412, 0.0, 0.0, 0.0, 1.0, 0.0]
	pub.publish(cam_info)

def main():
	global pub
	rospy.init_node("cali_remap_intrinsic_node")
	sub = rospy.Subscriber("/camera/color/camera_info_temp", CameraInfo, cb)
	pub = rospy.Publisher("/camera/color/camera_info", CameraInfo, queue_size=10)
	rospy.spin()

if __name__ == "__main__":
	main()
