import rospy
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from arm_operation.srv import joint_pose, joint_poseRequest, joint_poseResponse

class simple_demo:
	def __init__(self):
		rospy.init_node("simple_demo")
#		self.pub = rospy.Publisher("/set_pose/index_to_go", Int16, queue_size=10)
#		self.gripper_init     = rospy.ServiceProxy("/robotiq_finger_control_node/initial_gripper", Empty)
#		self.gripper_open     = rospy.ServiceProxy("/robotiq_finger_control_node/open_gripper", Empty)
#		self.gripper_close    = rospy.ServiceProxy("/robotiq_finger_control_node/close_gripper", Empty)
		self.conveyor_control = rospy.ServiceProxy("/arduino_control/conveyor_control", Empty)
#		self.ur5_goto_joint   = rospy.ServiceProxy("/ur5_control_service/ur_control/goto_joint_pose", joint_pose)
		rospy.loginfo("Initialization...")
#		self.gripper_init()
#		self.pub.publish(Int16(0))
		rospy.sleep(1.0)
		rospy.loginfo("Initialization completed, start processing!")
		self.process()
	def process(self):
		self.conveyor_control()
		rospy.sleep(7.0)
		rospy.sleep(15.0) # For putting mushroom
#		rospy.loginfo("Pick pipeapple...")
#		self.pick_pipeapple()
#		self.conveyor_control()
#		rospy.loginfo("Place pipeapple...")
#		self.place(0)
#		rospy.loginfo("Pick mushroom...")
#		self.pick_mushroom()
#		rospy.loginfo("Place mushroom...")
#		self.place(1)
#		rospy.loginfo("End process.")
		
	def pick_pipeapple(self):
		self.pub.publish(Int16(1))
		rospy.sleep(4.0)
		self.pub.publish(Int16(2))
		rospy.sleep(4.0)
		self.gripper_close()
		rospy.sleep(2.0)
		self.pub.publish(Int16(1))
		rospy.sleep(4.0)
		self.pub.publish(Int16(0))
		rospy.sleep(4.0)
		
	def pick_mushroom(self):
		self.pub.publish(Int16(1))
		rospy.sleep(3.5)
		self.pub.publish(Int16(2))
		rospy.sleep(3.0)
		self.gripper_close()
		rospy.sleep(2.0)
		self.pub.publish(Int16(3))
		rospy.sleep(3.0)
		
	'''
		place_type: 0 for pipeapple; 1 for mushroom
	'''
	def place(self, place_type):
		if place_type == 0:
			self.pub.publish(Int16(3))
			rospy.sleep(4.5)
			self.pub.publish(Int16(4))
			rospy.sleep(4.5)
			self.gripper_open()
			rospy.sleep(2.0)
			self.pub.publish(Int16(3))
			rospy.sleep(4.5)
		else:
			self.pub.publish(Int16(4))
			rospy.sleep(6.0)
			self.pub.publish(Int16(5))
			rospy.sleep(3.5)
			self.pub.publish(Int16(6))
			rospy.sleep(3.5)
			self.gripper_open()
			rospy.sleep(2.5)
			self.pub.publish(Int16(5))
			rospy.sleep(3.5)
			self.pub.publish(Int16(4))
			rospy.sleep(6.0)
		self.pub.publish(Int16(0))
		rospy.sleep(6.0)
def main():
	foo = simple_demo()

if __name__ == "__main__":
	main()
