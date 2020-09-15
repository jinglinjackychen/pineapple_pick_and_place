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
			if(mask_image.at<uchar>(i, j, 0) == 1){
				pineapple.at<uchar>(i, j, 0) = 255;
			}else if(mask_image.at<uchar>(i, j, 0) == 2){
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
	ros::init(argc, argv, "get_picture");
	ros::NodeHandle n;
	ros::ServiceClient prediction_client = n.serviceClient<task1_2::task1out>("/prediction");

	task1_2::task1out prediction_req;

	/*if (prediction_client.call(prediction_req)){
		ROS_INFO("plan_result: %s", prediction_req.response.info.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}*/

    Mat mask;
	Mat mask_channel[3];
	mask = imread("/home/arg/Desktop/mask.jpg",1);
	split(mask,mask_channel);

	//get_object_position(prediction_req.response.mask);
    //orientation_angle = get_object_orientation();
	get_image_mask(mask_channel[1]);

	ros::spinOnce();
	return 0;
}
