#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sign_detection/SignDataArray.h>

ros::Publisher pub_no_turn_right_trigger;
ros::Publisher pub_no_turn_left_trigger;
ros::Publisher pub_turn_right_trigger;
ros::Publisher pub_turn_left_trigger;
ros::Publisher pub_road_closed_trigger;

std_msgs::Bool no_turn_right_trigger_temp;
std_msgs::Bool no_turn_left_trigger_temp;
std_msgs::Bool turn_right_trigger_temp;
std_msgs::Bool turn_left_trigger_temp;
std_msgs::Bool road_closed_trigger_temp;


void recvSignData(const sign_detection::SignDataArrayConstPtr& msg)
{
  
  for(int i = 0; i<msg->data.size();i++){
    
    if(msg->data[i].sign_type == "No Turn Right"){
      if(!no_turn_right_trigger_temp.data){
	no_turn_right_trigger_temp.data = true;
	pub_no_turn_right_trigger.publish<std_msgs::Bool>(no_turn_right_trigger_temp);
      }else {
	no_turn_right_trigger_temp.data = false;
	pub_no_turn_right_trigger.publish<std_msgs::Bool>(no_turn_right_trigger_temp);
      }
    }
    
    if(msg->data[i].sign_type == "No Turn Left"){
      if(!no_turn_left_trigger_temp.data){
	no_turn_left_trigger_temp.data = true;
	pub_no_turn_left_trigger.publish<std_msgs::Bool>(no_turn_left_trigger_temp);
      }else {
	no_turn_left_trigger_temp.data = false;
	pub_no_turn_left_trigger.publish<std_msgs::Bool>(no_turn_left_trigger_temp);
      }
    }
    
    if(msg->data[i].sign_type == "One Way Right"){
      if(!turn_right_trigger_temp.data){
	turn_right_trigger_temp.data = true;
	pub_turn_right_trigger.publish<std_msgs::Bool>(turn_right_trigger_temp);
      }else {
	turn_right_trigger_temp.data = false;
	pub_turn_right_trigger.publish<std_msgs::Bool>(turn_right_trigger_temp);
      }
    }
    
    if(msg->data[i].sign_type == "One Way Left"){
      if(!turn_left_trigger_temp.data){
	turn_left_trigger_temp.data = true;
	pub_turn_left_trigger.publish<std_msgs::Bool>(turn_left_trigger_temp);
      }else {
	turn_left_trigger_temp.data = false;
	pub_turn_left_trigger.publish<std_msgs::Bool>(turn_left_trigger_temp);
      }
    }
    
    if(msg->data[i].sign_type == "Closed Road"){
      if(!road_closed_trigger_temp.data){
	road_closed_trigger_temp.data = true;
	pub_road_closed_trigger.publish<std_msgs::Bool>(road_closed_trigger_temp);
      }else {
	road_closed_trigger_temp.data = false;
	pub_road_closed_trigger.publish<std_msgs::Bool>(road_closed_trigger_temp);
      }
    }
    
  }
}  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turn_sign");
  ros::NodeHandle n;
  
  pub_no_turn_right_trigger = n.advertise<std_msgs::Bool>("no_turn_right_trigger", 1);
  pub_no_turn_left_trigger = n.advertise<std_msgs::Bool>("no_turn_left_trigger", 1);
  pub_turn_right_trigger = n.advertise<std_msgs::Bool>("turn_right_trigger", 1);
  pub_turn_left_trigger = n.advertise<std_msgs::Bool>("turn_left_trigger", 1);
  pub_road_closed_trigger = n.advertise<std_msgs::Bool>("road_closed_trigger", 1);
  
  ros::Subscriber sub_sign_data = n.subscribe("sign_data", 1, recvSignData);
  

  ros::spin();
}
