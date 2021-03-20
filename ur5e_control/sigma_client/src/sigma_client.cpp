#include <ros/ros.h>
#include <sigma_client/LocalizePart.h>
//#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sigma_client/PlanCartesianPath.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<sigma_client::LocalizePart>("localize_part");
    //cartesian_client_ = nh.serviceClient<sigma_client::PlanCartesianPath>("plan_path");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    sigma_client::LocalizePart srv;
    srv.request.base_frame = base_frame;

    ROS_INFO_STREAM("Requesting pose from: " << base_frame);

    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);

    move_target = srv.response.pose;
    move_target1 = srv.response.pose;
    //move_target.position.x += 0.5;
    //move_target.position.z += 1;
    // move_target.orientation.x = 0;
    // move_target.orientation.y = 0;
    // move_target.orientation.z = 0;
    // move_target.orientation.w = 1;
    // axis:Y 180
    move_target.orientation.x = -0.70710678 * move_target1.orientation.z + 0.70710678 * move_target1.orientation.w;
    move_target.orientation.y = -0.70710678 * move_target1.orientation.z - 0.70710678 * move_target1.orientation.w;
    move_target.orientation.z = 0.70710678 * move_target1.orientation.x + 0.70710678 * move_target1.orientation.y;
    move_target.orientation.w = -0.70710678 * move_target1.orientation.x + 0.70710678 * move_target1.orientation.y;

    //axis: Z -180
    ROS_INFO_STREAM("changed orientation:" << move_target);

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::MoveItErrorCode success;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setStartState(*move_group.getCurrentState());
    move_group.setPoseReferenceFrame(base_frame);
    move_group.setPoseTarget(move_target); 
    move_group.setPlanningTime(10);
    success = move_group.plan(plan);
    ROS_INFO("Visualizing plan (stateCatch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
    if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  
    move_group.execute(plan);
    move_group.setStartState(*move_group.getCurrentState());
    //move_group.move();

    sigma_client::PlanCartesianPath cartesian_srv;
    // cartesian_srv.request.pose = move_target;
    return ;
    // if (!cartesian_client_.call(cartesian_srv))
    // {
    //   ROS_ERROR("Could not plan for path");
    //   return;
    // }

    // Execute descartes-planned path directly (bypassing MoveIt)
    // ROS_INFO("Got cart path, executing");
    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = cartesian_srv.response.trajectory;
    // ac_.sendGoal(goal);
    // ac_.waitForResult();
    // ROS_INFO("Done");
  }
  
private:
  // Planning components
  ros::ServiceClient vision_client_;
  geometry_msgs::Pose move_target;
  geometry_msgs::Pose move_target1;
  //ros::ServiceClient cartesian_client_;
  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sigma_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle ("~");

  //如所描述的在这里，所述move_group.move()命令需要使用“异步”旋转器，以允许阻塞期间ROS消息的处理move()命令
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  while(ros::ok())
  {
    ROS_INFO("ScanNPlan node has been initialized");
    std::string base_frame;
    private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

    ScanNPlan app(nh);
    ros::Duration(.01).sleep();  // wait for the class to initialize
    //app.start(base_frame);
    app.start(base_frame);
    //ros::Duration(.1000).sleep();
    //ros::Duration(.1000).sleep();
    //ros::waitForShutdown();
    //ros::spinOnce();
    ros::Duration(.01).sleep();
  }
  return 0;
}
