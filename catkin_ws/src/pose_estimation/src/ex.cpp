#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h> // pcl::passThrough
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

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
  	std::vector<int> mapping;
  	pcl::removeNaNFromPointCloud(*load_cloud, *processed, mapping);
  	#ifdef VERBOSE
    		std::cout << "Remove nan: " << processed->points.size() << "\n";
  	#endif

  	pcl::PassThrough<pcl::PointXYZRGB> passx;
  	passx.setInputCloud (processed);
  	passx.setFilterFieldName ("x");
  	passx.setFilterLimits (0.0, 0.585);
  	//pass.setFilterLimitsNegative (true);
  	passx.filter (*filterx);

  	pcl::PassThrough<pcl::PointXYZRGB> passy;
  	passy.setInputCloud (filterx);
  	passy.setFilterFieldName ("y");
  	passy.setFilterLimits (-0.35, 0.25);
  	passy.filter (*filtery);

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "ex");
	ros::NodeHandle n;
	ros::Subscriber sub_point_cloud = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1000, cb_pc);

	while(pc.height==0){
		ros::spinOnce();
	}
    

	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
	// reader.read ("/home/jacky/code/flip_object/0.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << filtery->points.size () << " data points." << std::endl; //*

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud (filtery);
	vg.setLeafSize (0.002f, 0.002f, 0.002f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    pcl::PCDWriter writer;

  	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  	tree->setInputCloud (cloud_filtered);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  	ec.setClusterTolerance (0.02); // 2cm
  	ec.setMinClusterSize (500);
  	ec.setMaxClusterSize (25000);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud_filtered);
  	ec.extract (cluster_indices);

  	int j = 0;
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      		        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    		cloud_cluster->width = cloud_cluster->points.size ();
    		cloud_cluster->height = 1;
    		cloud_cluster->is_dense = true;
/*
            // PCL函数计算质心
	        Eigen::Vector4f centroid;					// 质心
	        pcl::compute3DCentroid(*cloud_cluster, centroid);	// 齐次坐标，（c0,c1,c2,1）
            std::cout << centroid(0) << "\t" << centroid(1) << "\t" << centroid(2) << std::endl;
*/
    		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    		std::stringstream ss;
    		ss << "cloud_cluster_" << j << ".pcd";
    		writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    		j++;
  	}

  	return (0);
}
