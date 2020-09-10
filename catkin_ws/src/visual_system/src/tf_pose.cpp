#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
float number = 0;
float HZ = 30.0;
float pose[7] = {0,0,0,0,0,0,0};

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_pose");
	ros::NodeHandle node;
	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
  	tf::TransformListener listener;
	tf::Quaternion q;
	ros::Duration(2.0).sleep();
  	ros::Rate rate(HZ);
  	while (node.ok()){
		if(number == HZ){
			ROS_INFO("X, Y, Z, q.x, q.y, q.z, q.w := %f %f %f %f %f %f %f", pose[0]/HZ, pose[1]/HZ, pose[2]/HZ, pose[3]/HZ, pose[4]/HZ, pose[5]/HZ, pose[6]/HZ);
			return 0;
		}
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/charuco", "camera_link", ros::Time(0), transform);
			//listener.lookupTransform("/map", "/tag_0", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		q = transform.getRotation();
		//ROS_INFO("X := %f ", transform.getOrigin().x());
		//ROS_INFO("q.x := %f ", q.x());
		pose[0] += transform.getOrigin().x();
		pose[1] += transform.getOrigin().y();
		pose[2] += transform.getOrigin().z();
		pose[3] += q.x();
		pose[4] += q.y();
		pose[5] += q.z();
		pose[6] += q.w();
		number++;
		rate.sleep();
	}
	return 0;
};
