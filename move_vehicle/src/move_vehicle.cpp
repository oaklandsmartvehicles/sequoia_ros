#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include "gps_conv.h"
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>

int num_o_wp,crs,col_x,col_y ;

ros::NodeHandle* node;
bool result_received = true;
int latch_gps;

int wp=0;
//Distance
double position_err;
ros::ServiceClient clear_costmap_srv;


tf::Vector3 utm_position; //relative to start point
tf::Vector3 target_global;


UTMCoords waypoint_utm[15];

UTMCoords ref_utm;
UTMCoords current_utm;
LatLon current_lla;

tf::TransformListener *listener_;


std_msgs::Float64 vehicle_heading;

std_msgs::Int8 move_mode;


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


void recvMODE(const std_msgs::Int8ConstPtr& msg)
{
  move_mode = *msg;
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

  int x_temp,y_temp,z_temp;

  if(move_mode.data == 0){
    x_temp = 30;
    y_temp = 2.0;
    z_temp = 0;

  }else if(move_mode.data== 1){
    x_temp = 30;
    y_temp = 13;
    z_temp = 0;

  }else if(move_mode.data == 2){
    x_temp = 30;
    y_temp = -8;
    z_temp = 0;
  }


  tf::Vector3 target_vehicle(x_temp, y_temp, z_temp);

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

  std_srvs::EmptyRequest req;
  std_srvs::EmptyResponse res;
  clear_costmap_srv.call(req, res);


}

void timerCallback(const ros::TimerEvent& event)
{
  //position_err=sqrt(pow(target_global.x()-utm_position.x(),2)+pow(target_global.y()-utm_position.y(),2));
//   if(position_err<=1.5)
//   {
    next_wp(1);
//   }
// dist_to_goal.data = position_err;
//   pub_dist.publish(dist_to_goal);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "move_vehicle");
  ros::NodeHandle node;

  latch_gps = 0;
  listener_ = new tf::TransformListener;
  move_mode.data = 0;

  double x,y;
  int zone,hemi;

  node.getParam("/TrueHeading", vehicle_heading.data);


//   ros::Subscriber subscribe_to_fix = node.subscribe("/fix",1,recvfix);
//   ros::Subscriber subscribe_to_heading = node.subscribe("/ekf_heading",1,recvHeading);

  ros::Subscriber subscribe_to_rviz = node.subscribe("/move_vehicle_mode",1,recvMODE);
//   ros::Subscriber subscribe_to_rviz = node.subscribe("/move_base_simple/goal",1,recvUMT);
  pub_goal = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  pub_dist = node.advertise<std_msgs::Float64>("/dist_to_goal", 1);

  clear_costmap_srv = node.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  ros::Timer control_timer;
  control_timer = node.createTimer(ros::Duration(0.3), timerCallback);
  ros::Duration(5).sleep();
//   next_wp(0);
  ros::spin();
  return 0;
}
