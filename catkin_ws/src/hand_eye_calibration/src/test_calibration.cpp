#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <arm_operation/joint_pose.h>
#include <arm_operation/target_pose.h>

/*
 *      3           2
 *             
 *      4           1      
 *            ROBOT_ARM
 */

int main(int argc, char** argv)
{
  std::vector<geometry_msgs::Point> traverse_point;
  geometry_msgs::Point p;
  p.x = -0.273f; p.y = 0.117f; p.z = 0.2f;
  traverse_point.push_back(p);
  p.x = -0.659f; traverse_point.push_back(p);
  p.y = -0.269f; traverse_point.push_back(p);
  p.x = -0.273f; traverse_point.push_back(p);
  arm_operation::joint_pose home_req, stby_req;
  arm_operation::target_pose target_req;
  home_req.request.joint[0] = 2.834;
  home_req.request.joint[1] = -2.629;
  home_req.request.joint[2] = 1.749;
  home_req.request.joint[3] = -1.482;
  home_req.request.joint[4] = -1.457;
  home_req.request.joint[5] = -0.187;
  stby_req.request.joint[0] = 3.121;
  stby_req.request.joint[1] = -1.709;
  stby_req.request.joint[2] = 2.065;
  stby_req.request.joint[3] = -2.319;
  stby_req.request.joint[4] = -1.731;
  stby_req.request.joint[5] = 0.159;
  target_req.request.factor = 0.5;
  target_req.request.target_pose.orientation.x = -0.707;
  target_req.request.target_pose.orientation.z = 0.707;
  ros::init(argc, argv, "test_calibration_node");
  ros::NodeHandle pnh("~");
  ros::ServiceClient joint_pose_client = pnh.serviceClient<arm_operation::joint_pose>\
                           ("/ur5_control_server/ur_control/goto_joint_pose");
  ros::ServiceClient target_pose_client = pnh.serviceClient<arm_operation::target_pose>\
                           ("/ur5_control_server/ur_control/goto_pose");
  ROS_INFO("Go to home");
  joint_pose_client.call(home_req); ros::Duration(2.0).sleep();
  for(int i=0; i<traverse_point.size(); ++i){
    ROS_INFO("Go to %d point", i+1);
    //joint_pose_client.call(stby_req); ros::Duration(3.0).sleep();
    target_req.request.target_pose.position.x = traverse_point[i].x;
    target_req.request.target_pose.position.y = traverse_point[i].y;
    target_req.request.target_pose.position.z = traverse_point[i].z;
    target_pose_client.call(target_req); ros::Duration(3.5).sleep();
  }
  ROS_INFO("Test complete, go home...");
  joint_pose_client.call(home_req); ros::Duration(2.0).sleep();
  return 0;
}
