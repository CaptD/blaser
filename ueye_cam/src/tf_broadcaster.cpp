#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  static tf::TransformBroadcaster br;

  tf::Transform transform_blaser;
  transform_blaser.setOrigin(tf::Vector3(-0.3079297, -0.0009062, 0.3244077));
  tf::Quaternion q_blaser(0.00404,-0.01097,-0.70753,0.70659);
  transform_blaser.setRotation(q_blaser);

  tf::Transform transform_nozzle;
  transform_nozzle.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
  tf::Quaternion q_nozzle;
  q_nozzle.setRPY(0, 0, 0);
  transform_nozzle.setRotation(q_nozzle);

  ros::Rate rate(30);
  while(node.ok()) {
    br.sendTransform(tf::StampedTransform(transform_blaser, ros::Time::now(), "foxbot_tool", "blaser"));
    br.sendTransform(tf::StampedTransform(transform_nozzle, ros::Time::now(), "foxbot_tool", "nozzle"));
    rate.sleep();
  }
  return 0;
};
