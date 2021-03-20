#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include <sigma_client/LocalizePart.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
//#include "sigma7/sigmaDevice.h"

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      sigma_sub = nh.subscribe<geometry_msgs::PoseStamped>("sigma7/pose", 1,
      &Localizer::number_callback, this);
      ROS_INFO("Vision node starting...");
      server_sigma = nh.advertiseService("localize_part", &Localizer::localizePart, this); //将服务公布给ROS主服务器
  }

  bool localizePart(sigma_client::LocalizePart::Request& req,
                  sigma_client::LocalizePart::Response& res)
  {
     // Read last message
      geometry_msgs::PoseStampedConstPtr p = last_msg_;
      //tf2_msgs::TFMessageConstPtr p = last_msg_;  
      //如果未订阅到主题，则不会运行服务器
      if (!p) 
        return false;
     
      res.pose = p->pose;
      return true;
      //tf::Transform master2slaveReachVec;
      //master2slaveReachVec.setOrigin(0,0,1.0);
      //tf::Transform ur_to_target = master2slaveReachVec * (p->transforms[0].transform);

      // tf::Transform world_to_target;
      // tf::transformMsgToTF(p->transforms[0].transform, world_to_target);
      // tf::StampedTransform ts;
      // try{
      //   ros::Time now = ros::Time::now();
      //   listener_.waitForTransform(req.base_frame, p->transforms[0].header.frame_id, 
      //                         now, ros::Duration(1.0));
      //   listener_.lookupTransform(req.base_frame, p->transforms[0].header.frame_id,
      //                        now, ts);
      //       }
      //  catch (tf::TransformException &ex) {
      // 	   ROS_ERROR("%s",ex.what());
      // 	   ros::Duration(1.0).sleep();
      // 	   //continue;
      // }
      // //对象姿势转换为目标帧,定义存放转换信息（平移，转动）的变量
      // tf::Transform ur_to_target;
      // ur_to_target = ts * world_to_target;
      //在服务响应中返回转换后的姿势
      //tf::poseTFToMsg(ur_to_target, res.pose);

      //return true;
  }
  void number_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      last_msg_ = msg;
      //ROS_INFO_STREAM(last_msg_->transforms[0].transform);
      ROS_INFO_STREAM(last_msg_ -> pose);
      //ROS_INFO("the number %d ",);
      //ROS_INFO("%s\n", s.data.c_str());
  }

  ros::Subscriber sigma_sub;
  geometry_msgs::PoseStampedConstPtr last_msg_;
  //tf2_msgs::TFMessageConstPtr last_msg_;
  ros::ServiceServer server_sigma;
  //tf::TransformListener listener_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sigma_vision");
  ros::NodeHandle nh;
  Localizer localizer(nh);
  ROS_INFO("Sigma is working...");
  ros::spin();
}