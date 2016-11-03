#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// #include <pcl/visualization/pcl_visualizer.h>

//#include <boost/thread/thread.hpp>

class BlaserScan
{
public:
	BlaserScan();
	void updatePointCloud(const sensor_msgs::PointCloud::ConstPtr& mcurrScan);
	void output_pcd(const std_msgs::Bool::ConstPtr& msg);
	//void generateMesh();
	sensor_msgs::PointCloud currScan;
	sensor_msgs::PointCloud2 allScan;
	tf::TransformListener tf_listener;
	
private:
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
	ros::ServiceServer ss;
	ros::Publisher pcl_pub;
	ros::Subscriber pcl_save_sub;
    //ros::Publisher mesh_pub;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //pcl::PolygonMesh mesh;
  
};

BlaserScan::BlaserScan() {
	pcl_sub = nh.subscribe("/camera_2/PointCloud", 10, &BlaserScan::updatePointCloud, this);
	pcl_save_sub = nh.subscribe("/save", 10, &BlaserScan::output_pcd, this);
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera_2/allScan",10);
    //mesh_pub = nh.advertise<pcl_msgs::PolygonMesh>("meshScan",10);
    //viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setBackgroundColor (0, 0, 0);
    //viewer->addPolygonMesh(mesh);
};

void BlaserScan::output_pcd(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data) {
        pcl::PointCloud<pcl::PointXYZI> output_pcl;
        pcl::PCLPointCloud2 cloud;
        pcl_conversions::toPCL(allScan, cloud); 
	std::cout<<"debug1"<<std::endl;
        pcl::fromPCLPointCloud2(cloud, output_pcl);
	std::cout<<"debug2"<<std::endl;
        pcl::io::savePCDFile<pcl::PointXYZI> ("world.pcd", output_pcl, true);
	std::cout<<"debug3"<<std::endl;
	} 
}

void BlaserScan::updatePointCloud(const sensor_msgs::PointCloud::ConstPtr& mcurrScan) {
	//currScan = *mcurrScan;
	const std::string& target_frame = "foxbot_base";
	//const std::string& target_frame = "base";
	//ros::Time t = ros::Time(0);
	try{
	  tf_listener.waitForTransform("blaser", "foxbot_base", mcurrScan->header.stamp, ros::Duration(2.0));
      //tf_listener.waitForTransform("blaser", "base", ros::Time::now(), ros::Duration(1.0));
      tf_listener.transformPointCloud(target_frame, mcurrScan->header.stamp, *mcurrScan, target_frame, currScan);
      
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
    sensor_msgs::PointCloud2 outScan_pc2;
    //sensor_msgs::PointCloud2 output;
    sensor_msgs::convertPointCloudToPointCloud2(currScan, outScan_pc2);
    pcl::concatenatePointCloud(allScan,outScan_pc2,allScan);

    /*
	  pcl::PointCloud<pcl::PointXYZ> pcl_cloud; 
    pcl::fromROSMsg(allScan,pcl_cloud);
	  
    std::cout << "pcl_cloud width" << pcl_cloud.width << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud_ptr = pcl_cloud.makeShared();
    std::cout << "pcl_cloud_ptr width" << pcl_cloud_ptr->width << std::endl;
    //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud_ptr);
    sor.setLeafSize(0.001f, 0.001f, 0.001f);
    sor.filter(*pcl_cloud_filtered_ptr);
    
    pcl::PointCloud<pcl::PointXYZ> allScan_pc;
    allScan_pc = *pcl_cloud_filtered_ptr;
    std::cout << "pcl_cloud_filtered_ptr ds width" << pcl_cloud_filtered_ptr->width << std::endl;
    pcl::toROSMsg(allScan_pc, allScan);
    
    std::cout << "pcl_cloud ds width" << allScan_pc.width << std::endl;
    //std::cout << allScan.header.frame_id << std::endl;
    */

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_downsampled = new pcl::PCLPointCloud2;

    // Convert to PCL data type
    pcl_conversions::toPCL(allScan, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.001, 0.001, 0.001);
    sor.filter(*cloud_downsampled);

    pcl::PCLPointCloud2ConstPtr cloud_downsampled_ptr(cloud_downsampled);
    pcl::PCLPointCloud2 cloud_bounded;
    Eigen::Vector4f minPoint;
    minPoint[0] = 1.0;
    minPoint[1] = -3.0;
    minPoint[2] = 0.0;
    Eigen::Vector4f maxPoint;
    maxPoint[0] = 7.0;
    maxPoint[1] = 3.0;
    maxPoint[2] = 1.0;
    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
    cropFilter.setInputCloud(cloud_downsampled_ptr);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.filter(cloud_bounded);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_bounded, output);
    allScan = output;
	pcl_pub.publish(allScan);

    /*
    // reconstruction 
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromPCLPointCloud2(cloud_filtered, cloud_xyz);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_xyz_ptr;
    pcl_cloud_xyz_ptr = cloud_xyz.makeShared();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(pcl_cloud_xyz_ptr);
    ne.setRadiusSearch(0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid(*pcl_cloud_xyz_ptr, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);
    //std::cout << "normal estimation complete" << std::endl;
    //std::cout << "reverse normals' direction" << std::endl;

    for(size_t i = 0; i < cloud_normals->size(); ++i){
      cloud_normals->points[i].normal_x *= -1;
      cloud_normals->points[i].normal_y *= -1; 
      cloud_normals->points[i].normal_z *= -1;
    }

    //std::cout << "combine points and normals" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*pcl_cloud_xyz_ptr, *cloud_normals, *cloud_smoothed_normals);

    //std::cout << "begin poisson reconstruction" << std::endl;
    pcl::PolygonMesh mesh;
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(mesh);
    //viewer->updatePolygonMesh(mesh);
    if (!viewer->addPolygonMesh(mesh)) {
      viewer->updatePolygonMesh(mesh);
      viewer->spinOnce();
      std::cout << "update not add" << std::endl;
    };
    pcl_msgs::PolygonMesh mesh_msg;
    pcl_conversions::fromPCL(mesh, mesh_msg);
    mesh_pub.publish(mesh_msg);
    */



};

int main(int argc, char** argv){
  ros::init(argc, argv, "blaser_scan_red");
  ros::start(); // start the node resource managers (communication, time, etc)
  
  BlaserScan bs;
  
  ros::spin();

  return 0;
};
