#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h> // pcl::ModelCoefficients
#include <pcl/io/pcd_io.h> // pcl::io::savePCDFileASCII, pcl::io::savePCDFileBinary
#include <pcl/point_types.h> // pcl::PointXYZ
#include <pcl/filters/passthrough.h> // pcl::passThrough
#include <pcl/common/transforms.h> // pcl::transformPointCloud
#include <pcl/sample_consensus/method_types.h> // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h> // pcl::SACMODEL_PLANE
#include <pcl/segmentation/sac_segmentation.h> // pcl::SACSegmentation
#include <pcl/registration/icp.h> // pcl::IterativeClosestPoint
#include <pcl/filters/voxel_grid.h> // pcl::VoxelGrid
#include <visual_system/helper.h>
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// SRV
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <visual_system/get_xyz.h>
#include <visual_system/get_pc.h>
#include <visual_system/pc_is_empty.h>
#include <visual_system/check_valid.h>
#include <visual_system/get_surface_feature.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
// Includes listed here
#define PI 3.14159

using namespace cv;
using namespace std;

double fx = 669.1729736328125;
double cx = 652.1881713867188;
double fy = 669.1729736328125;
double cy = 366.0839538574219;

sensor_msgs::PointCloud2 pc;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterx (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtery (new pcl::PointCloud<pcl::PointXYZRGB>);


void cb_pc(const sensor_msgs::PointCloud2 pointcloud){
	while (pointcloud.height == 0){
		ROS_INFO("Incorrect PointCloud2 vector size, ignore...");
		return;
	}
	pc = pointcloud;
	// sensor_msgs::PointCloud2 to PCL type
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(pointcloud,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*load_cloud);

	// Remove NaN points
  	vector<int> mapping;
  	pcl::removeNaNFromPointCloud(*load_cloud, *processed, mapping);
  	#ifdef VERBOSE
    		cout << "Remove nan: " << processed->points.size() << "\n";
  	#endif

  	pcl::PassThrough<pcl::PointXYZRGB> passx;
  	passx.setInputCloud (processed);
  	passx.setFilterFieldName ("x");
  	passx.setFilterLimits (0.0, 0.67);
  	//pass.setFilterLimitsNegative (true);
  	passx.filter (*filterx);

  	pcl::PassThrough<pcl::PointXYZRGB> passy;
  	passy.setInputCloud (filterx);
  	passy.setFilterFieldName ("y");
  	passy.setFilterLimits (-0.35, 0.25);
  	passy.filter (*filtery);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "goto_pose");
	ros::NodeHandle n;
	ros::Subscriber sub_point_cloud = n.subscribe("/camera/color/image_rect_color", 50, cb_pc);


	ros::spinOnce();
	return 0;
}
