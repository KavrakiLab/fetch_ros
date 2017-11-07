/* Author: Chamzas Constantinos */

//ROS stuff
#include <geometry_msgs/TransformStamped.h> 
#include <tf_conversions/tf_eigen.h> 

#include <gpd/GraspConfigList.h>
#include <gpd/GraspConfig.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>



class getGrasps
{
public:
  getGrasps(ros::NodeHandle& n) : n_(n)
  {
    sub_ = n_.subscribe("/detect_grasps/clustered_grasps", 3,&getGrasps::callback, this);
    graspBest_ = gpd::GraspConfig();
    graspBest_.score.data = 0;
  }
  void update(gpd::GraspConfig graspCurr){

       graspBest_ = graspCurr;
       geometry_msgs::Vector3 app = graspBest_.approach;
	   geometry_msgs::Vector3 bin = graspBest_.binormal;
	   geometry_msgs::Vector3 axi = graspBest_.axis;
	   tf::Matrix3x3 rot = tf::Matrix3x3(app.x,bin.x,axi.x,app.y,bin.y,axi.y,app.z,bin.z,axi.z);

	   double roll, pitch,yaw;
	   rot.getRPY(roll,pitch,yaw);

	   graspRot_ = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
	   approach_ = app;
  }

  void callback(const gpd::GraspConfigList& input)
  {
    graspList_ = input;
   // ROS_INFO("RECEIVED GRASPS");
    for (int i = 0;i< graspList_.grasps.size();i++){
	    gpd::GraspConfig graspCurr;
	    graspCurr = graspList_.grasps[i];
	    
	    if ((graspBest_.score.data) < (graspCurr.score.data)){
            update(graspCurr);
	    }
    }
  } 

  geometry_msgs::Point getBestGrasp()
  {
      ROS_INFO("Sample Chosen: %f,%f,%f",graspBest_.bottom.x,graspBest_.bottom.y,graspBest_.bottom.z);
      return graspBest_.bottom;
  }
  geometry_msgs::Vector3 getApproach()
  {
     return approach_;
  }
  std_msgs::Header getBestGraspHeader()
  {
     return graspHeader_;
  }
  geometry_msgs::Quaternion getGraspRot()
  {
     return graspRot_;
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  gpd::GraspConfigList graspList_;
  gpd::GraspConfig graspBest_;
  std_msgs::Header graspHeader_;
  geometry_msgs::Quaternion graspRot_;
  geometry_msgs::Vector3 approach_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch_pickUp_tutorial");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "arm";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  getGrasps* grasps = new getGrasps(n);
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual: world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Raw pointers are frequently used to refer to the planning kgroup for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPoseReferenceFrame("/head_camera_rgb_optical_frame");
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("/head_camera_rgb_optical_frame");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "setPoseReferenceFrame %s",move_group.getPoseReferenceFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  //waiting for a grasp to be received
  std::cout<<"Waiting For Grasps to Be Received To Continue Press Enter"<<std::endl;
  std::cin.ignore();
  // Planning to a Pose goal
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  
  geometry_msgs::Point graspPoint ;
  graspPoint = grasps->getBestGrasp();

  geometry_msgs::PoseStamped target_pose_stamped1;
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose3;
  geometry_msgs::Vector3 approach;
  approach= grasps->getApproach();
  target_pose1.orientation= geometry_msgs::Quaternion(grasps->getGraspRot());
  target_pose3.orientation= geometry_msgs::Quaternion(grasps->getGraspRot());

  target_pose1.position.x = graspPoint.x-0.3*approach.x;
  target_pose1.position.y = graspPoint.y-0.3*approach.y;
  target_pose1.position.z = graspPoint.z-0.3*approach.z; 
  target_pose3.position.x = graspPoint.x-0.1*approach.x;
  target_pose3.position.y = graspPoint.y-0.1*approach.y;
  target_pose3.position.z = graspPoint.z-0.1*approach.z;
  
  target_pose_stamped1.pose = target_pose1;
  target_pose_stamped1.header = grasps->getBestGraspHeader();
  move_group.setPoseTarget(target_pose1);


  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.28;
  target_pose2.position.y = -0.7;
  target_pose2.position.z = 1.0; 
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  visual_tools.publishAxisLabeled(target_pose1,"PreGrasp");
  visual_tools.publishAxisLabeled(target_pose2,"poseFIXED");
  visual_tools.publishAxisLabeled(target_pose3,"Grasp");
  visual_tools.trigger();
  //visual_tools.publishAxisLabeled(target_pose_stamped1,"stamped_pose1");
  std::cout<<"To plan the path press Enter"<<std::endl;
  std::cin.ignore();
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = move_group.plan(my_plan);
  std::cout<<"To execute the path press Enter"<<std::endl;
  std::cin.ignore();
  move_group.move();

  std::cout<<"To Open the Gripper press Enter"<<std::endl;
  std::cin.ignore();
  moveit::planning_interface::MoveGroupInterface move_group_gripper(PLANNING_GROUP_GRIPPER);
  move_group_gripper.setJointValueTarget("r_gripper_finger_joint",0.05);
  move_group_gripper.setJointValueTarget("l_gripper_finger_joint",0.05);
  success = move_group_gripper.plan(my_plan);
  move_group_gripper.move();
  std::cout<<"To Close the Gripper press Enter"<<std::endl;
  std::cin.ignore();
  move_group_gripper.setJointValueTarget("r_gripper_finger_joint",0.02);
  move_group_gripper.setJointValueTarget("l_gripper_finger_joint",0.02);
  success = move_group_gripper.plan(my_plan);
  move_group_gripper.move();
  std::cin.ignore();
  

  }
