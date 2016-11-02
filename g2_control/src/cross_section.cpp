#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_handlers.h>
//#include <boost/thread/thread.hpp>

class CrossSection
{
public:
	CrossSection();
	void updatePointCloud(const sensor_msgs::PointCloud2::ConstPtr& mallScan);
	void showCrossSection(const sensor_msgs::PointCloud::ConstPtr& mcurrScan);
	//void generateMesh();
	sensor_msgs::PointCloud currScan;
	sensor_msgs::PointCloud2 allScan;
	tf::TransformListener tf_listener;
	
private:
	ros::NodeHandle nh;
	ros::Subscriber pcl_front_sub;
	ros::Subscriber pcl_rear_sub;
	pcl::PointXYZ min_p, max_p;
	//Eigen::Vector3f min_pt.fill(0.0);
	//Eigen::Vector3f max_pt.fill(0.0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;  
};

CrossSection::CrossSection() {
	pcl_front_sub = nh.subscribe("/camera_1/PointCloud", 10, &CrossSection::showCrossSection, this);
	pcl_rear_sub = nh.subscribe("/camera_2/allScan", 10, &CrossSection::updatePointCloud, this);
    //min_pt[]
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    //viewer->addPolygonMesh(mesh);
    //viewer.show
};

void CrossSection::showCrossSection(const sensor_msgs::PointCloud::ConstPtr& mcurrScan) {
	const std::string& target_frame = "foxbot_base";
	try{
	  tf_listener.waitForTransform("blaser", "foxbot_base", ros::Time::now(), ros::Duration(1.0));
      //tf_listener.waitForTransform("blaser", "base", ros::Time::now(), ros::Duration(1.0));
      tf_listener.transformPointCloud(target_frame, ros::Time(0), *mcurrScan, target_frame, currScan);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
    sensor_msgs::PointCloud2 currScan_pc2;
    sensor_msgs::convertPointCloudToPointCloud2(currScan, currScan_pc2);
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(currScan_pc2, *cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromPCLPointCloud2(*cloud, cloud_xyz);
    pcl::getMinMax3D(cloud_xyz, min_p, max_p);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (cloudPtr, 0, 255, 0);
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2 (*cloud, *cloud_new);
    std::cout << *cloud << std::endl;
    if (!viewer->addPointCloud<pcl::PointXYZ>(cloud_new, "red",0)) {
    	viewer->updatePointCloud<pcl::PointXYZ>(cloud_new, "red");
    	viewer->spinOnce();
    }
}

void CrossSection::updatePointCloud(const sensor_msgs::PointCloud2::ConstPtr& mallScan) {
	sensor_msgs::PointCloud allScan_new;
	sensor_msgs::PointCloud temp;
	sensor_msgs::convertPointCloud2ToPointCloud(*mallScan,temp);
	const std::string& target_frame = "foxbot_base";
	try{
	  tf_listener.waitForTransform("blaser", "foxbot_base", ros::Time::now(), ros::Duration(1.0));
      tf_listener.transformPointCloud(target_frame, ros::Time(0), temp, target_frame, allScan_new);   
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
    sensor_msgs::PointCloud2 allScan_pc2;
    sensor_msgs::convertPointCloudToPointCloud2(allScan_new, allScan_pc2);
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(allScan_pc2, *cloud);

    pcl::PCLPointCloud2 cloud_bounded;
    Eigen::Vector4f minPoint;
    minPoint[0] = min_p.x - 0.0005;
    minPoint[1] = min_p.y - 0.0005;
    minPoint[2] = min_p.z - 0.0005;
    Eigen::Vector4f maxPoint;
    maxPoint[0] = max_p.x + 0.0005;
    maxPoint[1] = max_p.y + 0.0005;
    maxPoint[2] = max_p.z + 0.0005;
    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.filter(cloud_bounded);

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
  ros::init(argc, argv, "cross_section");
  ros::start(); // start the node resource managers (communication, time, etc)
  
  CrossSection cs;
  
  ros::spin();

  return 0;
};
