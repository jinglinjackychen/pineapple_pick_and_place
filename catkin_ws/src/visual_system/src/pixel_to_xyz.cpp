#include <fstream>
#include <opencv2/opencv.hpp> // cv
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// SRV
#include <std_srvs/Empty.h>
#include <visual_system/get_xyz.h>
#include <visual_system/get_image.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

bool verbose;
const int LENGTH = 224;
const int X_MIN = 246; // 208
const int Y_MIN = 93; // 21
std::string frame_name;
std::vector<double> intrinsic; // [fx, fy, cx, cy]
cv::Rect myROI(0, 0, 590, 720);
cv::Mat last_color, last_depth;
cv_bridge::CvImagePtr color_img_ptr, depth_img_ptr;
ros::Publisher pub_pc, pub_color, pub_depth;

void callback_sub(const sensor_msgs::ImageConstPtr& color_image, \
		  const sensor_msgs::ImageConstPtr& depth_image, \
                  const sensor_msgs::CameraInfoConstPtr& cam_info);
bool callback_service(visual_system::get_xyz::Request &req,
                      visual_system::get_xyz::Response &res);
bool callback_get_image(visual_system::get_image::Request &req,
                        visual_system::get_image::Response &res);

int main(int argc, char** argv)
{
  intrinsic.resize(4);
  ros::init(argc, argv, "pixel_to_xyz01");
  ros::NodeHandle nh, pnh("~");
  if(pnh.getParam("verbose", verbose)) verbose = false;
  if(1){
    ROS_WARN("Publish data for debug");
    pub_pc = pnh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    pub_color = pnh.advertise<sensor_msgs::Image>("crop_color", 1);
    pub_depth = pnh.advertise<sensor_msgs::Image>("crop_depth", 1);
  }
  else
    ROS_WARN("Not publish pointcloud");
  // Message filter: color image, depth image, color camera info
  message_filters::Subscriber<sensor_msgs::Image> color_image_sub(nh, "zed/zed_node/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "zed/zed_node/depth/depth_registered", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "zed/zed_node/rgb/camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, \
                                   sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub, depth_image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback_sub, _1, _2, _3));

  ros::ServiceServer service = pnh.advertiseService("pixel_to_xyz01", callback_service);
  ros::ServiceServer image_service = pnh.advertiseService("get_image", callback_get_image);
  ros::spin();
  return 0;
}


void callback_sub(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
              const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  frame_name = color_image->header.frame_id;
  intrinsic[0] = cam_info->K[0]; // fx
  intrinsic[1] = cam_info->K[4]; // fy
  intrinsic[2] = cam_info->K[2]; // cx
  intrinsic[3] = cam_info->K[5]; // cy
  try{
    color_img_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }  
  try{
    depth_img_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
 
  pcl::PointCloud<pcl::PointXYZRGB> pc;


//          580 ~ 1169
  for(int x=0; x<=1279; ++x){ // 1280  640
    for(int y=0; y<=719; ++y){ // 720  480
      // More readable
      auto depth = depth_img_ptr->image.at<float>(cv::Point(x, y));
      geometry_msgs::Point p;

//      ROS_INFO("%f ", (float)depth);

      // Equivalent
      /*
      if(depth!=0){
        pcl::PointXYZRGB p;
        p.z = depth * 0.001f;
        p.x = (x-intrinsic[2])/intrinsic[0]*p.z;
        p.y = (y-intrinsic[3])/intrinsic[1]*p.z;
        p.r = color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
        p.g = color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
        p.b = color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
        pc.points.push_back(p);
      }*/

//      depth=(depth-255)*(-1);
//      ROS_INFO("%d ", (int)depth);

      if(depth<0.80){
        p.z = depth /* * 0.001f*/; // Represent in 1 mm
//        ROS_INFO("%f ", (float)p.z);
        p.x = (x-intrinsic[2])/intrinsic[0]*p.z; // x = (u-cx)/fx*z
        p.y = (y-intrinsic[3])/intrinsic[1]*p.z; // y = (v-cy)/fy*z
        if(1){
          pcl::PointXYZRGB pc_p;
          pc_p.x = p.x; pc_p.y = p.y; pc_p.z = p.z;
          pc_p.r = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[2];
          pc_p.g = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[1];
          pc_p.b = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[0];
          pc.push_back(pc_p);
        }
      } else{
        geometry_msgs::Point p;
        p.x = p.y = p.z = std::numeric_limits<double>::quiet_NaN();
      }
      pc.height = 1; pc.width = pc.points.size();
    } // End for(y)
  } // End for(x) 



  if(1){
    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(pc, pc_out);
    pc_out.header.frame_id = "base_link01";
    pub_pc.publish(pc_out);
    ROS_INFO("%d ", (int)pc_out.width);


    cv::Mat crop_color = color_img_ptr->image(myROI), 
            crop_depth = depth_img_ptr->image(myROI);
    cv_bridge::CvImage crop_color_cv_bridge(color_img_ptr->header, color_img_ptr->encoding, crop_color),
                       crop_depth_cv_bridge(depth_img_ptr->header, depth_img_ptr->encoding, crop_depth);
    pub_color.publish(crop_color_cv_bridge.toImageMsg());
    pub_depth.publish(crop_depth_cv_bridge.toImageMsg());
  }
}

/*bool callback_service(visual_system::get_xyz::Request &req,
                      visual_system::get_xyz::Response &res){
  int idx = req.point[0]*LENGTH+req.point[1];
  if(idx>=pixel_to_xyz.size()){
    ROS_WARN("Invalid request index, please remember to subtract 208 for x and 21 for y index");
    return false;
  }
  geometry_msgs::Point p = pixel_to_xyz[idx];
  res.result = p;
  if(verbose){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_name;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position = p;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
    marker.color.b = marker.color.a = 1.0; // BLUE
  }
  ROS_INFO("\nRequest: pixel(%d, %d)\n\
Response: point(%f, %f, %f)", 
            req.point[0], req.point[1], p.x, p.y, p.z);
  return true;
}*/

bool callback_service(visual_system::get_xyz::Request  &req,
                      visual_system::get_xyz::Response &res){
  geometry_msgs::Point p;
  int pixel_x = X_MIN+req.point[0],
      pixel_y = Y_MIN+req.point[1];
  auto depth = last_depth.at<unsigned short>(cv::Point(req.point[0], req.point[1]));
  if(depth==0){
    p.x = p.y = p.z = std::numeric_limits<double>::quiet_NaN();
  } else{
    p.z = depth * 0.001f; // 1mm
    p.x = (pixel_x-intrinsic[2])/intrinsic[0]*p.z; // x = (u-cx)/fx*z
    p.y = (pixel_y-intrinsic[3])/intrinsic[1]*p.z; // y = (v-cy)/fy*z
  }
  res.result = p;
  ROS_INFO("\nRequest: pixel(%d, %d)\nResponse: point(%f, %f, %f)", req.point[1], req.point[0], p.x, p.y, p.z);
  return true;
}

bool callback_get_image(visual_system::get_image::Request &req,
                        visual_system::get_image::Response &res){
  cv::Mat crop_color = color_img_ptr->image(myROI), 
          crop_depth = depth_img_ptr->image(myROI);
  last_color = crop_color; last_depth = crop_depth;
  cv_bridge::CvImage crop_color_cv_bridge(color_img_ptr->header, color_img_ptr->encoding, crop_color),
                     crop_depth_cv_bridge(depth_img_ptr->header, depth_img_ptr->encoding, crop_depth);
  res.crop_color_img = *crop_color_cv_bridge.toImageMsg();
  res.crop_depth_img = *crop_depth_cv_bridge.toImageMsg();
  return true;
}
