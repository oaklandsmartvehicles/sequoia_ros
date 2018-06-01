#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include "gps_conv.h"
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

int num_o_wp,crs,col_x,col_y ;

ros::NodeHandle* node;
bool result_received = true;
int latch_gps;

int wp=0;
//Distance
double position_err;


tf::Vector3 utm_position; //relative to start point
tf::Vector3 target_global;


UTMCoords waypoint_utm[15];

UTMCoords ref_utm;
UTMCoords current_utm;
LatLon current_lla;

tf::TransformListener *listener_;


std_msgs::Float64 vehicle_heading;


//Outpot GPS coordinates of a 2d Nav Goal point selected on RViz
void recvUMT(const geometry_msgs::PoseStamped& msg) 
{
LatLon output;
output = ref_utm + tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0);
// ROS_INFO("position : %f, %f", utm_position.x(), utm_position.y());
// ROS_INFO("goal : %f, %f", msg.pose.position.x, msg.pose.position.y);

}

double ref_lat; //Latitude of the start location of audibot
double ref_lon; //Latitude of the start location of audibot



ros::Publisher pub_goal;
ros::Publisher pub_dist;

geometry_msgs::PoseStamped goal_msg;
std_msgs::Float64 dist_to_goal;


void recvfix(const sensor_msgs::NavSatFixConstPtr& msg)
{
 // Set reference coordinates
  if (latch_gps == 0){
  LatLon ref_point(*msg);
  ref_utm = UTMCoords(ref_point);  
  latch_gps = 1;
  }
  
  UTMCoords current_utm_temp(*msg);
  LatLon current_lla_temp(*msg);
  
  current_utm = current_utm_temp;
  current_lla = current_lla_temp;
  utm_position = current_utm-ref_utm;

}

void recvHeading(const std_msgs::Float64ConstPtr& msg)
{
 // Set reference coordinates
  vehicle_heading = *msg;
}



void next_wp(int x){
  
   // Project output lines from camera into vehicle frame and publish as obstacles
  tf::StampedTransform transform;
  try {
    listener_->lookupTransform("map", "base_footprint", ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    return;
  }


  tf::Vector3 target_vehicle(30, 0, 0);
  target_global = transform * target_vehicle;
  
  //goal_msg.header.stamp = ros::Time::now();
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = target_global.x();
  goal_msg.pose.position.y = target_global.y();
  goal_msg.pose.orientation.x = transform.getRotation().x();
  goal_msg.pose.orientation.y = transform.getRotation().y();
  goal_msg.pose.orientation.z = transform.getRotation().z();
  goal_msg.pose.orientation.w = transform.getRotation().w();
  pub_goal.publish(goal_msg);
  

}

void timerCallback(const ros::TimerEvent& event)
{
  //position_err=sqrt(pow(target_global.x()-utm_position.x(),2)+pow(target_global.y()-utm_position.y(),2));
//   if(position_err<=1.5)
//   {
    next_wp(1);
//   }
  dist_to_goal.data = position_err;
  pub_dist.publish(dist_to_goal);
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "move_vehicle");
  ros::NodeHandle node;
  
  latch_gps = 0;
  listener_ = new tf::TransformListener;

  double x,y;
  int zone,hemi;
  
  node.getParam("/TrueHeading", vehicle_heading.data);


  ros::Subscriber subscribe_to_fix = node.subscribe("/fix",1,recvfix);
  ros::Subscriber subscribe_to_heading = node.subscribe("/ekf_heading",1,recvHeading);
  ros::Subscriber subscribe_to_rviz = node.subscribe("/move_base_simple/goal",1,recvUMT);
  pub_goal = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  pub_dist = node.advertise<std_msgs::Float64>("/dist_to_goal", 1);

  ros::Timer control_timer;
  control_timer = node.createTimer(ros::Duration(3.0), timerCallback);
  ros::Duration(5).sleep();
//   next_wp(0);
  ros::spin();
  return 0;
}
