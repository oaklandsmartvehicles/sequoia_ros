#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sign_detection/SignDataArray.h>

ros::Publisher pub_stop_trigger;

std_msgs::Bool stop_trigger_temp;


void recvSignData(const sign_detection::SignDataArrayConstPtr& msg)
{
  
  for(int i = 0; i<msg->data.size();i++){
    
    if(msg->data[i].sign_type == "Stop Sign"){
      if(!stop_trigger_temp.data){
	  if(2 > msg->data[i].sign_position.x){
	  stop_trigger_temp.data = true;
	  pub_stop_trigger.publish<std_msgs::Bool>(stop_trigger_temp);
	  }
      }else {
	  if(2 <= msg->data[i].sign_position.x){
	  stop_trigger_temp.data = false;
	  pub_stop_trigger.publish<std_msgs::Bool>(stop_trigger_temp);
	  }
      }
    }
  }
}  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stop_sign");
  ros::NodeHandle n;
  
  
  pub_stop_trigger = n.advertise<std_msgs::Bool>("stop_sign_trigger", 1);
  ros::Subscriber sub_sign_data = n.subscribe("sign_data", 1, recvSignData);
  

  ros::spin();
}