#include <ros/ros.h>
#include <tf/transform_listener.h>
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
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <foxbot/robot_JogCartesian.h>

class EdgeFollow
{
public:
	EdgeFollow();
	void extractEdge(const sensor_msgs::PointCloud::ConstPtr& mcurrScan);
	char getch();
	sensor_msgs::PointCloud currScan;
	tf::TransformListener tf_listener;

private:
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
    ros::Publisher target_pub;
    ros::ServiceClient client;
    bool initilaized;
    bool first_frame;
    geometry_msgs::Point32 target_point;
    Eigen::VectorXf smooth_score;
    Eigen::MatrixXf smoothed_points;
    Eigen::MatrixXf::Index max_index;
    Eigen::VectorXd max_index_buffer;
    int target_index;
    pcl::PointCloud<pcl::PointXYZ> cloud_data;
    Eigen::MatrixXf trajectory;
    tf::StampedTransform transform;
};

EdgeFollow::EdgeFollow() {
	pcl_sub = nh.subscribe("/camera_2/PointCloud", 10, &EdgeFollow::extractEdge, this);
    target_pub = nh.advertise<sensor_msgs::PointCloud>("target",10);
    client = nh.serviceClient<foxbot::robot_JogCartesian>("/foxbot/robot_JogCartesian");
    initilaized = false;
    first_frame = true;
    trajectory = Eigen::MatrixXf::Zero(30,3);
};

char EdgeFollow::getch() {
    char buf = 0;
    
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
            
    return (buf);
}

void EdgeFollow::extractEdge(const sensor_msgs::PointCloud::ConstPtr& mcurrScan) {
	if (!initilaized) {
		if(first_frame) {
			const std::string& target_frame = "foxbot_base";
			//const std::string& target_frame = "base";
			//ros::Time t = ros::Time(0);
			try{
			  tf_listener.waitForTransform("blaser", "foxbot_base", ros::Time::now(), ros::Duration(2.0));
		      //tf_listener.waitForTransform("blaser", "base", ros::Time::now(), ros::Duration(1.0));
		      tf_listener.transformPointCloud(target_frame, ros::Time(0), *mcurrScan, target_frame, currScan);  
		    }
		    catch (tf::TransformException &ex) {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		      //continue;
		    }
		    sensor_msgs::PointCloud2 outScan_pc2;
		    sensor_msgs::convertPointCloudToPointCloud2(currScan, outScan_pc2);

		    // Container for original & filtered data
		    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
		    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		    pcl::PCLPointCloud2* cloud_downsampled = new pcl::PCLPointCloud2;

		    // Convert to PCL data type
		    pcl_conversions::toPCL(outScan_pc2, *cloud);

		    // Perform the actual filtering
		    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		    sor.setInputCloud (cloudPtr);
		    sor.setLeafSize (0.01, 0.01, 0.01);
		    sor.filter (*cloud_downsampled);

		    pcl::PCLPointCloud2ConstPtr cloud_downsampled_ptr(cloud_downsampled);
		    pcl::PCLPointCloud2 cloud_bounded;
		    Eigen::Vector4f minPoint;
		    minPoint[0] = 1.0;
		    minPoint[1] = -4.0;
		    minPoint[2] = -3.0;
		    Eigen::Vector4f maxPoint;
		    maxPoint[0] = 10.0;
		    maxPoint[1] = 4.0;
		    maxPoint[2] = 3.0;
		    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
		    cropFilter.setInputCloud(cloud_downsampled_ptr);
		    cropFilter.setMin(minPoint);
		    cropFilter.setMax(maxPoint);
		    cropFilter.filter(cloud_bounded);

		    
		    pcl::fromPCLPointCloud2(cloud_bounded, cloud_data);

		    //std::cout << "point cloud height:" << cloud_bounded.height << std::endl;
		    //std::cout << "point cloud width:" << cloud_bounded.width << std::endl;
		    //std::cout << "point cloud x:" << cloud_data.points[0].x << std::endl;
		    
		    smoothed_points.resize(cloud_data.width-7, 3);
		    for (int index = 0; index < cloud_data.width-7; index++) {
		    	
		    	smoothed_points(index,0) = (cloud_data.points[index].x+15*cloud_data.points[index+1].x+67*cloud_data.points[index+2].x+111*cloud_data.points[index+3].x+67*cloud_data.points[index+4].x+15*cloud_data.points[index+5].x+cloud_data.points[index+6].x)/277;
		    	smoothed_points(index,1) = (cloud_data.points[index].y+15*cloud_data.points[index+1].y+67*cloud_data.points[index+2].y+111*cloud_data.points[index+3].y+67*cloud_data.points[index+4].y+15*cloud_data.points[index+5].y+cloud_data.points[index+6].y)/277;
		    	smoothed_points(index,2) = (cloud_data.points[index].z+15*cloud_data.points[index+1].z+67*cloud_data.points[index+2].z+111*cloud_data.points[index+3].z+67*cloud_data.points[index+4].z+15*cloud_data.points[index+5].z+cloud_data.points[index+6].z)/277;

		    }
		    //std::cout << smoothed_points << std::endl;
		    /*
		    int num_neighbor = 7;
		    smooth_score.resize(cloud_data.width-13);
		    Eigen::Vector3f temp;
		    for (int i = 0; i < cloud_data.width-13; i++ ) {
		    	temp(0) = 0.0;
		    	temp(1) = 0.0;
		    	temp(2) = 0.0;
		    	for (int j = i; j < i + num_neighbor; j++) {
		    		temp(0) = temp(0) + smoothed_points(i+3,0)-smoothed_points(j,0);
		    		temp(1) = temp(1) + smoothed_points(i+3,1)-smoothed_points(j,1);
		    		temp(2) = temp(2) + smoothed_points(i+3,2)-smoothed_points(j,2);
		    	}
		        smooth_score(i) = temp.norm()/smoothed_points.row(i+3).norm();
		        //std::cout << temp.norm() << " " << smoothed_points.row(i+3).norm() << std::endl;
		    }
		    */
		    int num_neighbor = 7;
		    smooth_score.resize(cloud_data.width-13);
		    Eigen::Vector3f temp;
		    for (int i = 0; i < cloud_data.width-13; i++ ) {
		        smooth_score(i) = -smoothed_points(i+6,2)-smoothed_points(i,2)+2*smoothed_points(i+3,2);
		        //std::cout << temp.norm() << " " << smoothed_points.row(i+3).norm() << std::endl;
		    }
		    smooth_score.maxCoeff(&max_index);
		    std::cout << max_index << std::endl;
		    max_index_buffer.resize(1);
		    max_index_buffer(0) = max_index;
		    target_index = 0;    

		    // Convert to ROS data type
		    sensor_msgs::PointCloud output;
		    std_msgs::Header header;
		    header.stamp = ros::Time::now();
		    header.frame_id = "foxbot_base";
		    output.header = header;
		    target_point.x = cloud_data.points[max_index+3].x;
		    target_point.y = cloud_data.points[max_index+3].y;
		    target_point.z = cloud_data.points[max_index+3].z;
		    output.points.resize(1);
		    output.points[0] = target_point;
		    //pcl_conversions::fromPCL(cloud_bounded, output);
			target_pub.publish(output);
			first_frame = 0;
		} else {
			// read keyboard input
			char c = getch();
			//std::cout << c << std::endl;
			if (c == '\n') {
				initilaized = true;
				max_index = max_index_buffer(target_index);
				
				tf::StampedTransform transform;
				bool succeed = false;
				while(!succeed) {
					succeed = true;
					try{
						tf_listener.lookupTransform("/foxbot_base", "/nozzle", ros::Time(0), transform);
						//std::cout << transform.getOrigin().x() << std::endl;
					}
					catch (tf::TransformException ex){
						succeed = false;
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
					}
				}
				std::cout << "nozzle:" << transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << std::endl;
				std::cout << "blaser:" << cloud_data.points[max_index+3].x << ", " << cloud_data.points[max_index+3].y << ", " << cloud_data.points[max_index+3].z <<std::endl;
				for (int i = 0; i < 30; i++) {
					trajectory(i,0) = ((29-i)*transform.getOrigin().x()+(i+1)*cloud_data.points[max_index+3].x)/30.0;
					trajectory(i,1) = ((29-i)*transform.getOrigin().y()+(i+1)*cloud_data.points[max_index+3].y)/30.0;
					trajectory(i,2) = ((29-i)*transform.getOrigin().z()+(i+1)*cloud_data.points[max_index+3].z)/30.0;
					
				}
				//std::cout << trajectory << std::endl;
			} else if (c == 'd') {
                int index_size = max_index_buffer.size();
                if (index_size == target_index+1) {
                	max_index_buffer.conservativeResize(index_size+1);
					smooth_score(max_index) = 0;
				    smooth_score.maxCoeff(&max_index);
				    max_index_buffer(index_size) = max_index;
				    target_index++;
				} else {
					target_index++;
					max_index = max_index_buffer(target_index);
				}
			    sensor_msgs::PointCloud output;
			    std_msgs::Header header;
			    header.stamp = ros::Time::now();
			    header.frame_id = "foxbot_base";
			    output.header = header;
			    target_point.x = cloud_data.points[max_index+3].x;
			    target_point.y = cloud_data.points[max_index+3].y;
			    target_point.z = cloud_data.points[max_index+3].z;
			    output.points.resize(1);
			    output.points[0] = target_point;
			    //pcl_conversions::fromPCL(cloud_bounded, output);
				target_pub.publish(output);

                
			} else if (c == 'a') {
				target_index--;
				if (target_index < 0) target_index == 0;
				max_index = max_index_buffer(target_index);
				sensor_msgs::PointCloud output;
			    std_msgs::Header header;
			    header.stamp = ros::Time::now();
			    header.frame_id = "foxbot_base";
			    output.header = header;
			    target_point.x = cloud_data.points[max_index+3].x;
			    target_point.y = cloud_data.points[max_index+3].y;
			    target_point.z = cloud_data.points[max_index+3].z;
			    output.points.resize(1);
			    output.points[0] = target_point;
			    //pcl_conversions::fromPCL(cloud_bounded, output);
				target_pub.publish(output);
			}

		}
	} else {
		std::cout << "auto started." << std::endl;
		
		bool succeed = false;
		while(!succeed) {
			succeed = true;
			try{
				tf_listener.lookupTransform("/foxbot_base", "/nozzle", ros::Time(0), transform);		

				std::cout << transform.getOrigin().x() << std::endl;
			}
			catch (tf::TransformException ex){
				succeed = false;
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}
		foxbot::robot_JogCartesian srv;
		srv.request.x = trajectory(0, 0) - transform.getOrigin().x();
		srv.request.y = trajectory(0, 1) - transform.getOrigin().y();
		srv.request.z = 0.0;
		srv.request.rx = 0.0;
		srv.request.ry = 0.0;
		srv.request.rz = 0.0;
		if (client.call(srv))
	    {
	        ROS_INFO("Succeed calling service");
	    }
	    else
	    {
	    	ROS_ERROR("Failed to call service");
	    }
		const std::string& target_frame = "foxbot_base";
		//const std::string& target_frame = "base";
		//ros::Time t = ros::Time(0);
		try{
		  tf_listener.waitForTransform("blaser", "foxbot_base", ros::Time::now(), ros::Duration(2.0));
	      //tf_listener.waitForTransform("blaser", "base", ros::Time::now(), ros::Duration(1.0));
	      tf_listener.transformPointCloud(target_frame, ros::Time(0), *mcurrScan, target_frame, currScan);  
	    }
	    catch (tf::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      //continue;
	    }
	    sensor_msgs::PointCloud2 outScan_pc2;
	    sensor_msgs::convertPointCloudToPointCloud2(currScan, outScan_pc2);

	    // Container for original & filtered data
	    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	    pcl::PCLPointCloud2* cloud_downsampled = new pcl::PCLPointCloud2;

	    // Convert to PCL data type
	    pcl_conversions::toPCL(outScan_pc2, *cloud);

	    // Perform the actual filtering
	    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	    sor.setInputCloud (cloudPtr);
	    sor.setLeafSize (0.01, 0.01, 0.01);
	    sor.filter (*cloud_downsampled);

	    pcl::PCLPointCloud2ConstPtr cloud_downsampled_ptr(cloud_downsampled);
	    pcl::PCLPointCloud2 cloud_bounded;
	    Eigen::Vector4f minPoint;
	    minPoint[0] = trajectory(29,0) - 0.04;
	    minPoint[1] = trajectory(29,1) - 0.03;
	    minPoint[2] = trajectory(29,2) - 0.03;
	    Eigen::Vector4f maxPoint;
	    maxPoint[0] = trajectory(29,0) + 0.04;
	    maxPoint[1] = trajectory(29,1) + 0.03;
	    maxPoint[2] = trajectory(29,2) + 0.03;
	    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
	    cropFilter.setInputCloud(cloud_downsampled_ptr);
	    cropFilter.setMin(minPoint);
	    cropFilter.setMax(maxPoint);
	    cropFilter.filter(cloud_bounded);

	    pcl::PointCloud<pcl::PointXYZ> cloud_data;
	    pcl::fromPCLPointCloud2(cloud_bounded, cloud_data);

	    

	    //std::cout << "point cloud height:" << cloud_bounded.height << std::endl;
	    std::cout << "point cloud width:" << cloud_bounded.width << std::endl;
	    //std::cout << "point cloud x:" << cloud_data.points[0].x << std::endl;
	    /*
	    
	    smoothed_points.resize(cloud_data.width-4, 3);
	    for (int index = 0; index < cloud_data.width-4; index++) {
	    	//cloud_bounded.data[index]
	    	
	    	smoothed_points(index,0) = (cloud_data.points[index].x+15*cloud_data.points[index+1].x+67*cloud_data.points[index+2].x+111*cloud_data.points[index+3].x+67*cloud_data.points[index+4].x+15*cloud_data.points[index+5].x+cloud_data.points[index+6].x)/277;
	    	smoothed_points(index,1) = (cloud_data.points[index].y+15*cloud_data.points[index+1].y+67*cloud_data.points[index+2].y+111*cloud_data.points[index+3].y+67*cloud_data.points[index+4].y+15*cloud_data.points[index+5].y+cloud_data.points[index+6].y)/277;
	    	smoothed_points(index,2) = (cloud_data.points[index].z+15*cloud_data.points[index+1].z+67*cloud_data.points[index+2].z+111*cloud_data.points[index+3].z+67*cloud_data.points[index+4].z+15*cloud_data.points[index+5].z+cloud_data.points[index+6].z)/277;
	    	

	    	smoothed_points(index,0) = (cloud_data.points[index].x+3*cloud_data.points[index+1].x+5*cloud_data.points[index+2].x+3*cloud_data.points[index+3].x+cloud_data.points[index+4].x)/13.0;
	    	smoothed_points(index,1) = (cloud_data.points[index].y+3*cloud_data.points[index+1].y+5*cloud_data.points[index+2].y+3*cloud_data.points[index+3].y+cloud_data.points[index+4].y)/13.0;
	    	smoothed_points(index,2) = (cloud_data.points[index].z+3*cloud_data.points[index+1].z+5*cloud_data.points[index+2].z+3*cloud_data.points[index+3].z+cloud_data.points[index+4].z)/13.0;

	    }

	    int num_neighbor = 5;
	    smooth_score.resize(cloud_data.width-8);
	    for (int i = 0; i < (cloud_data.width-8); i++ ) {
	        smooth_score(i) = -smoothed_points(i+4,2)-smoothed_points(i,2)+2*smoothed_points(i+2,2);
	    }
	    
	    //Eigen::MatrixXf::Index max_index;
	    smooth_score.maxCoeff(&max_index);

	    */

	    smooth_score.resize(cloud_data.width);
	    for (int i = 0; i < (cloud_data.width); i++ ) {
	        smooth_score(i) = -(pow(cloud_data.points[i].x-trajectory(29,0),2.0)+pow(cloud_data.points[i].y-trajectory(29,1),2.0)+5*pow(cloud_data.points[i].z-trajectory(29,2),2.0));
	    }

	    smooth_score.maxCoeff(&max_index);
	    
	    for (int i = 0; i < 29; i++) {
	    	trajectory(i,0) = trajectory(i+1,0);
	    	trajectory(i,1) = trajectory(i+1,1);
	    	trajectory(i,2) = trajectory(i+1,2);
	    }
	    trajectory(29,0) = cloud_data.points[max_index].x;
	    trajectory(29,1) = cloud_data.points[max_index].y;
	    trajectory(29,2) = cloud_data.points[max_index].z;

	    

	    // Convert to ROS data type
	    sensor_msgs::PointCloud output;
	    geometry_msgs::Point32 target_point;
	    std_msgs::Header header;
	    header.stamp = ros::Time::now();
	    header.frame_id = "foxbot_base";
	    output.header = header;
	    target_point.x = cloud_data.points[max_index].x;
	    target_point.y = cloud_data.points[max_index].y;
	    target_point.z = cloud_data.points[max_index].z;
	    output.points.resize(1);
	    output.points[0] = target_point;
	    //pcl_conversions::fromPCL(cloud_bounded, output);
		target_pub.publish(output);
	}
	
	

};

int main(int argc, char** argv){
  ros::init(argc, argv, "edge_following");
  ros::start(); // start the node resource managers (communication, time, etc)
  
  EdgeFollow ef;

  ros::spin();

  return 0;
};
