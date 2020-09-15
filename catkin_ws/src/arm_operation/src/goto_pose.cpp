#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> // PointXYZ
#include <arm_operation/target_pose.h>
#include <arm_operation/joint_pose.h>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ur_kin.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <task1_2/task1out.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#define PI 3.14159

using namespace cv;
using namespace std;

sensor_msgs::PointCloud2 pc;
sensor_msgs::JointState js;
cv_bridge::CvImagePtr prediction_mask_ptr;

int point_position[2], pineapple_position[2];
double orientation_angle = 0;

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw){
    Eigen::AngleAxisd rollAngle(roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw,Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle*pitchAngle*rollAngle;
    return q;
}

void cb_pc(const sensor_msgs::PointCloud2 pointcloud){
	while (pointcloud.height == 0){
		ROS_INFO("Incorrect PointCloud2 vector size, ignore...");
		return;
	}
	pc = pointcloud;
}

void cb_js(const sensor_msgs::JointState joint_pose){
	while (joint_pose.name.size() != 6){
		ROS_INFO("Incorrect JointState vector size, ignore...");
		return;
	}
	js = joint_pose;
}

static inline bool ContoursSortFun(vector<Point> contour1,vector<Point> contour2)  
{  
    return (contourArea(contour1) > contourArea(contour2));  
}

void get_object_position(const sensor_msgs::Image& mask_image_msg)
{
	Mat mask_image;
    prediction_mask_ptr = cv_bridge::toCvCopy(mask_image_msg);  //sensor_msgs::Imag >>> cvimage
  	prediction_mask_ptr->image.copyTo(mask_image);              //sensor_msgs::Imag >>> cvimage

	Mat pineapple(mask_image.rows, mask_image.cols, CV_8UC1, Scalar(0)), pick(mask_image.rows, mask_image.cols, CV_8UC1, Scalar(0));
	vector<vector<Point>> pick_contours,pineapple_contours;//定義發現的輪廓
	vector<Vec4i> pick_hierarchy,pineapple_hierarchy;//層次
	Vec3b  color_red=Vec3b(0,0,255), color_blue=Vec3b(255,0,0), color_green=Vec3b(0,255,0);
	int i, j;

	for (i = 0; i < mask_image.rows; i++){
		for(j = 0; j < mask_image.cols; j++){
			if(mask_image.at<uchar>(i, j, 0) == 2){
				pineapple.at<uchar>(i, j, 0) = 255;
			}else if(mask_image.at<uchar>(i, j, 0) == 1){
				pick.at<uchar>(i, j, 0) = 255;
			}
		}
	}
	findContours(pick, pick_contours, pick_hierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0));                //findContours
    findContours(pineapple, pineapple_contours, pineapple_hierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0)); //findContours

    sort(pick_contours.begin(),pick_contours.end(),ContoursSortFun);            //sort contours
    sort(pineapple_contours.begin(),pineapple_contours.end(),ContoursSortFun);  //sort contours

	printf("pick_contours : %d \n",pick_contours.size());

	printf("pineapple_contours : %d \n",pineapple_contours.size());

	//計算影象矩
	vector<Moments> pick_mu(1);
	pick_mu[0] = moments(pick_contours[0], false);
	//計算影象的質心
	vector<Point2i> pick_mc(1);
	pick_mc[0] = Point2i(pick_mu[0].m10 / pick_mu[0].m00, pick_mu[0].m01 / pick_mu[0].m00);
	printf("pick_X : %d pick_Y : %d \n",pick_mc[0].x,pick_mc[0].y);
	point_position[0]=pick_mc[0].x;
	point_position[1]=pick_mc[0].y;

	//計算影象矩
	vector<Moments> pineapple_mu(1);
	pineapple_mu[0] = moments(pineapple_contours[0], false);
	//計算影象的質心
	vector<Point2i> pineapple_mc(1);
	pineapple_mc[0] = Point2i(pineapple_mu[0].m10 / pineapple_mu[0].m00, pineapple_mu[0].m01 / pineapple_mu[0].m00);
	printf("pineapple_X : %d pineapple_Y : %d \n",pineapple_mc[0].x,pineapple_mc[0].y);
	pineapple_position[0]=pineapple_mc[0].x;
	pineapple_position[1]=pineapple_mc[0].y;
}

void get_image_mask(Mat mask_image)
{
	Mat mask_image_rgb(mask_image.rows, mask_image.cols, CV_8UC3, Scalar(0)), pick(mask_image.rows, mask_image.cols, CV_8UC1, Scalar(0));
	int i, j;

	for (i = 0; i < mask_image.rows; i++){
		for(j = 0; j < mask_image.cols; j++){
			if(mask_image.at<uchar>(i, j, 0) == 0){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 255;
			}else if(mask_image.at<uchar>(i, j, 0) == 1){
				mask_image_rgb.at<Vec3b>(i, j)[1] = 255;
			}else if(mask_image.at<uchar>(i, j, 0) == 2){
				mask_image_rgb.at<Vec3b>(i, j)[2] = 255;
			}
		}
	}

	imwrite("/home/arg/Desktop/mask1.jpg",mask_image_rgb);
}

double get_object_orientation(){
    int new_pineapple_X, new_pineapple_Y, new_pick_X, new_pick_Y;
    int reference_vector[2], orientation_vector[2];
    double cos_value;
    new_pineapple_Y = 1279-pineapple_position[0];
    new_pineapple_X = 719-pineapple_position[1];
    new_pick_Y = 1279-point_position[0];
    new_pick_X = 719-point_position[1];
    printf("NEW_pineapple_X : %d NEW_pineapple_Y : %d \n",new_pineapple_X,new_pineapple_Y);
    printf("NEW_pick_X : %d NEW_pick_Y : %d \n",new_pick_X,new_pick_Y);
    reference_vector[0] = 0;
    reference_vector[1] = 1;
    orientation_vector[0] = new_pick_X - new_pineapple_X;
    orientation_vector[1] = new_pick_Y - new_pineapple_Y;
    cos_value = (orientation_vector[1])/(sqrt((orientation_vector[0]*orientation_vector[0])+(orientation_vector[1]*orientation_vector[1])));
    printf("angle : %lf \n",(acos(cos_value)*180.0/PI));
    if(orientation_vector[0] > 0){
        return -acos(cos_value);
    }
    else{
        return acos(cos_value);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goto_pose");
	ros::NodeHandle n;
	ros::Subscriber sub_point_cloud = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1000, cb_pc);
	ros::Subscriber sub_joint_pose = n.subscribe("/joint_states", 1000, cb_js);
    ros::Publisher gripper_control = n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 1000);
	ros::ServiceClient goto_pose_client = n.serviceClient<arm_operation::target_pose>("/ur5_control_server/ur_control/goto_pose");
    ros::ServiceClient goto_joint_pose_client = n.serviceClient<arm_operation::joint_pose>("/ur5_control_server/ur_control/goto_joint_pose");
	ros::ServiceClient prediction_client = n.serviceClient<task1_2::task1out>("/prediction");
	arm_operation::target_pose tp, home_pose;
    arm_operation::joint_pose joint_pose1,joint_pose2,joint_pose3,home_joint_pose;
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output close, open;

//------------------------home pose------------------------------------
    home_pose.request.target_pose.position.x = -0.3;
    home_pose.request.target_pose.position.y = -0.1;
    home_pose.request.target_pose.position.z = 0.5;
    home_pose.request.target_pose.orientation.x = 0.383;
    home_pose.request.target_pose.orientation.y = 0.0;
    home_pose.request.target_pose.orientation.z = -0.924;
    home_pose.request.target_pose.orientation.w = 0.0;
    home_pose.request.factor = 0.5;
//------------------------gripper close and open-----------------------
    close.rACT = 1;
    close.rGTO = 1;
    close.rATR = 0;
    close.rPR = 255;
    close.rSP = 255;
    close.rFR = 150;

    open.rACT = 1;
    open.rGTO = 1;
    open.rATR = 0;
    open.rPR = 0;
    open.rSP = 255;
    open.rFR = 150;

    gripper_control.publish(open);
    ros::Duration(1.0).sleep();
//---------------------------------------------------------------------

	task1_2::task1out prediction_req;
	tf::TransformListener listener(ros::Duration(10));
	while(pc.height==0){
		ros::spinOnce();
	}

	if (prediction_client.call(prediction_req)){
		ROS_INFO("plan_result: %s", prediction_req.response.info.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	get_object_position(prediction_req.response.mask);
    orientation_angle = get_object_orientation();
	float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pc.data[(point_position[0]*16)+(point_position[1]*20480)], sizeof(float));
    memcpy(&Y, &pc.data[(point_position[0]*16)+(point_position[1]*20480)+4], sizeof(float));
    memcpy(&Z, &pc.data[(point_position[0]*16)+(point_position[1]*20480)+8], sizeof(float));

	printf("%f %f %f \n",X,Y,Z);

	geometry_msgs::PointStamped pick_point;
	pick_point.header.stamp=ros::Time();
	pick_point.header.frame_id="zed_left_camera_frame";
	pick_point.point.x=X;
	pick_point.point.y=Y;
	pick_point.point.z=Z;
	geometry_msgs::PointStamped pick_point_robot;
        try{
		    listener.transformPoint("base_link",pick_point,pick_point_robot);
        }catch (tf::TransformException &ex){
          	ROS_ERROR("%s",ex.what());
          	ros::Duration(1.0).sleep();
    	}
	printf("%lf %lf %lf \n",pick_point_robot.point.x,pick_point_robot.point.y,pick_point_robot.point.z);

    Eigen::Quaterniond quat = euler2Quaternion(0, (PI / 2), (-PI + orientation_angle));
	tp.request.target_pose.position.x = pick_point_robot.point.x;
	tp.request.target_pose.position.y = pick_point_robot.point.y;
	tp.request.target_pose.position.z = pick_point_robot.point.z + 0.20;
    tp.request.target_pose.orientation.x = quat.x();
	tp.request.target_pose.orientation.y = quat.y();
	tp.request.target_pose.orientation.z = quat.z();
	tp.request.target_pose.orientation.w = quat.w();
    tp.request.factor = 0.5;
/* --------------------------------------------------------------------------------- */
    Mat mask;
	Mat mask_channel[3];
	mask = imread("/home/arg/Desktop/mask.jpg",1);
	split(mask,mask_channel);


	get_image_mask(mask_channel[1]);
/* --------------------------------------------------------------------------------- */
    if (goto_pose_client.call(tp)){
		ROS_INFO("plan_result: %s", tp.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(1.0).sleep();
/* --------------------------------------------------------------------------------- */
	tp.request.target_pose.position.z = -0.02;
    if (goto_pose_client.call(tp)){
		ROS_INFO("plan_result: %s", tp.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(1.0).sleep();
/* --------------------------------------------------------------------------------- */
    gripper_control.publish(close);
    ros::Duration(1.0).sleep();
/* --------------------------------------------------------------------------------- */
    if (goto_pose_client.call(home_pose)){
		ROS_INFO("plan_result: %s", home_pose.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(1.0).sleep();
/* --------------------------------------------------------------------------------- */
    joint_pose1.request.joint[0] = 1.0834956169128418;
    joint_pose1.request.joint[1] = -1.983957592641012;
    joint_pose1.request.joint[2] = 1.5927839279174805;
    joint_pose1.request.joint[3] = -1.437695328389303;
    joint_pose1.request.joint[4] = -1.5677512327777308;
    joint_pose1.request.joint[5] = 0.31808868050575256;
/* --------------------------------------------------------------------------------- */
    if (goto_joint_pose_client.call(joint_pose1)){
		ROS_INFO("plan_result: %s", joint_pose1.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(0.75).sleep();
/* --------------------------------------------------------------------------------- */
    joint_pose2.request.joint[0] = 1.0864437818527222;
    joint_pose2.request.joint[1] = -1.2719643751727503;
    joint_pose2.request.joint[2] = 1.4249343872070312;
    joint_pose2.request.joint[3] = -2.0389040152179163;
    joint_pose2.request.joint[4] = -1.5762346426593226;
    joint_pose2.request.joint[5] = 0.3184242248535156;
/* --------------------------------------------------------------------------------- */
    if (goto_joint_pose_client.call(joint_pose2)){
		ROS_INFO("plan_result: %s", joint_pose2.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(0.75).sleep();
/* --------------------------------------------------------------------------------- */
    joint_pose3.request.joint[0] = 1.1011263132095337;
    joint_pose3.request.joint[1] = -1.0356457869159144;
    joint_pose3.request.joint[2] = 1.4292044639587402;
    joint_pose3.request.joint[3] = -1.9168275038348597;
    joint_pose3.request.joint[4] = -1.6827495733844202;
    joint_pose3.request.joint[5] = 0.31834033131599426;
/* --------------------------------------------------------------------------------- */
    if (goto_joint_pose_client.call(joint_pose3)){
		ROS_INFO("plan_result: %s", joint_pose3.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(0.75).sleep();
/* --------------------------------------------------------------------------------- */
    gripper_control.publish(open);
    ros::Duration(0.75).sleep();
/* --------------------------------------------------------------------------------- */
    if (goto_joint_pose_client.call(joint_pose2)){
		ROS_INFO("plan_result: %s", joint_pose2.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(0.75).sleep();
/* --------------------------------------------------------------------------------- */
    if (goto_joint_pose_client.call(joint_pose1)){
		ROS_INFO("plan_result: %s", joint_pose1.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(1.0).sleep();
/* --------------------------------------------------------------------------------- */
//3.127866268157959, -2.087594811116354, 1.7123093605041504, -1.6890042463885706, -1.5448411146747034, -0.03432161012758428
    home_joint_pose.request.joint[0] = 3.127866268157959;
    home_joint_pose.request.joint[1] = -2.087594811116354;
    home_joint_pose.request.joint[2] = 1.7123093605041504;
    home_joint_pose.request.joint[3] = -1.6890042463885706;
    home_joint_pose.request.joint[4] = -1.5448411146747034;
    home_joint_pose.request.joint[5] = -0.03432161012758428;
    if (goto_joint_pose_client.call(home_joint_pose)){
		ROS_INFO("plan_result: %s", home_joint_pose.response.plan_result.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
    ros::Duration(1.5).sleep();
/* --------------------------------------------------------------------------------- */

	ros::spinOnce();
	return 0;
}
