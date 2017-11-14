/* Author: Chamzas Constantinos */
// ROS stuff
#include <tf2_ros/transform_listener.h>

#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <control_msgs/GripperCommand.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
void printTF(tf::Transform tra)
{
    tf::Quaternion quat;
    tf::Point orig;
    quat = tra.getRotation();
    orig = tra.getOrigin();
    std::cout << "Quat:[X,Y,Z,W]=[" << quat.getX() << ',' << quat.getY() << "," << quat.getZ()<<","<<quat.getW()<< std::endl;
    
    std::cout << "Orig:[X,Y,Z]=[" << orig.getX() << ',' << orig.getY() << "," << orig.getZ()<<std::endl;
}

class getGrasps
{
public:
    getGrasps(ros::NodeHandle& n) : n_(n), tfListener_(tfBuffer_)
    {
        sub_ = n_.subscribe("/detect_grasps/clustered_grasps", 3, &getGrasps::callback, this);
        reset();
    }
    tf::Transform graspToTF(gpd::GraspConfig& graspCurr, const std::string& baseFrame,
                            const std::string& graspFrame)
    {
        geometry_msgs::Vector3 app = graspCurr.approach;
        geometry_msgs::Vector3 bin = graspCurr.binormal;
        geometry_msgs::Vector3 axi = graspCurr.axis;
        tf::Matrix3x3 rot =
            tf::Matrix3x3(app.x, bin.x, axi.x, app.y, bin.y, axi.y, app.z, bin.z, axi.z);

        // The bottom point on the surface object has been chosen as origin
        tf::Point origin;
        tf::pointMsgToTF(graspCurr.bottom, origin);
        tf::Transform headToGrasp = tf::Transform(rot, origin);

        tf::StampedTransform worldToHead;
        tf::transformStampedMsgToTF(lookup(baseFrame, graspFrame), worldToHead);

        tf::Transform graspTF = worldToHead * headToGrasp;
        return graspTF;
    }

    geometry_msgs::TransformStamped lookup(const std::string& base, const std::string& target)
    {
        try
        {
            return tfBuffer_.lookupTransform(base, target, ros::Time(0));
        }
        catch (tf2::LookupException ex)
        {
            ROS_INFO("GraspClient::lookup:%s,%s):%s", base.c_str(), target.c_str(), ex.what());
        }
    }
    void reset(){


        ROS_INFO("GraspClient:: graspBest_ was reseted");
        graspBest_.score.data =0;
    }

    void callback(const gpd::GraspConfigList& graspList)
    {
        // Iterating over all differrent grasps
        for (int i = 0; i < graspList.grasps.size(); i++)
        {
            gpd::GraspConfig graspCurr = graspList.grasps[i];
            tf::Transform graspCurrTF =
                graspToTF(graspCurr, "base_link", graspList.header.frame_id);

            // MUST CHANGE
            // THIS MAGIC NUMBER MUST BE THE HEIGHT OF THE TABLE
            // Naively choose the grasp with the best Score

            if (((graspBest_.score.data) < (graspCurr.score.data)) &&
                (graspCurrTF.getOrigin().getZ() > 0.84))
            {
                graspTF_ = graspCurrTF;
                graspBest_ = graspCurr;
                printTF(graspTF_);
            }
        }
    }

    tf::Transform getGraspTF()
    {
        return graspTF_;
    };

    geometry_msgs::Pose getGraspPoseMsg(double offset = 0)
    {
        geometry_msgs::Pose graspMsgPose;
        // Adding the approach(X axis of Basis Column) in the oppossite direction
        tf::Vector3 graspAppScale = graspTF_.getBasis().getColumn(0) * (-offset);

        tf::quaternionTFToMsg(graspTF_.getRotation(), graspMsgPose.orientation);
        tf::pointTFToMsg(graspTF_.getOrigin() + graspAppScale, graspMsgPose.position);
        // MUST CHANGE
        // HEURISTIC APPORACH TO CHOOSE GOOD GRASPS SHOULD CHANGE IN THE FUTURE
        
        if (graspMsgPose.position.z < 0.88){ graspMsgPose.position.z = 0.88;}

        return graspMsgPose;
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    gpd::GraspConfig graspBest_;
    tf::Transform graspTF_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fetch_pickUp_tutorial");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    static const std::string BASE_FRAME = "/base_link";

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>armClient(
        "arm_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for arm client to Start...");
    armClient.waitForServer();
    
    ROS_INFO("Arm Client started!");

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>gripperClient(
        "gripper_controller/gripper_action", true);

    ROS_INFO("Waiting for gripper Client to Start...");
    gripperClient.waitForServer();
    ROS_INFO("Gripper Client  started!");

    getGrasps* grasps = new getGrasps(n);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual: world" scene
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group.setPoseReferenceFrame(BASE_FRAME);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    ROS_INFO_NAMED("GraspDemo", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("GraspDemo", "setPoseReferenceFrame %s",
                   move_group.getPoseReferenceFrame().c_str());

    ROS_INFO_NAMED("GraspDemo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // waiting for a grasp to be received
    geometry_msgs::Pose preGrasp; 
    geometry_msgs::Pose finGrasp;
    
    bool attemp_grasp =true;
    bool execute_pre_grasp;
    bool execute_grasp;
    bool success_plan;
    int num_tries; 
    //This is the main Loop that tries to execute a grasp from start to end
    while (attemp_grasp){
        execute_pre_grasp = false;
        num_tries = 0;
        //This loop tries to execute the pregrasping
        while(!execute_pre_grasp){

            grasps->reset();
            visual_tools.deleteAllMarkers();

            std::cout << "To visualize the current Best Grasp  Press Enter" << std::endl;
            std::cin.ignore();

            preGrasp = grasps->getGraspPoseMsg(0.3);
            finGrasp = grasps->getGraspPoseMsg(0.14);

            visual_tools.publishAxisLabeled(preGrasp, "preGrasp");
            visual_tools.publishAxisLabeled(finGrasp, "Grasp");
            visual_tools.trigger();
            std::cout << "To plan the path for preGrap press Enter" << std::endl;
            std::cin.ignore();

            // Setting  a Pose goal for the move_group. In this case the arm
            move_group.setPoseTarget(preGrasp);
            // Now, we call the planner to compute the plan .
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            success_plan = move_group.plan(my_plan);
           

            //To execute the plan we must use the execute(). Note move() both plans and executes
            if (success_plan){
                std::cout << "To execute the path for preGrasp press Enter" << std::endl;
                std::cin.ignore();
                num_tries = 0;
                while (num_tries<5&&!execute_pre_grasp){
                        execute_pre_grasp = move_group.execute(my_plan);
                        num_tries+=1;
                }
            }
            else {
                ROS_INFO_NAMED("graspDemo", "Failed to find plan. Retrying ...");
            }

        }

        execute_grasp = false; 
        //This loop will try to execute the final grasping. Can only be executed if pre_grasping was a success.
        while(!execute_grasp&&execute_pre_grasp){
            
            std::cout << "To find the trajectory for  Grasp Press Enter" << std::endl;
            std::cin.ignore();
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(preGrasp);
            waypoints.push_back(finGrasp);
            moveit_msgs::RobotTrajectory trajectory;
            const double jthr= 0;
            const double eef_step= 0.3;
            double fraction =  0;
            num_tries = 0;
            while (fraction <0.8&& num_tries<5){
                fraction = move_group.computeCartesianPath(waypoints, eef_step, jthr, trajectory,false);
                num_tries+=1;
            }
            if (fraction <0.8){execute_pre_grasp = false;}
            if (execute_pre_grasp){

                std::cout << "To Open Gripper Press Enter" << std::endl;
                std::cin.ignore();
                ROS_INFO("Openning gripper ..");
                control_msgs::GripperCommandGoal gripperGoal;
                control_msgs::GripperCommand gripperCommand;
                gripperCommand.position = 0.1; 
                gripperCommand.max_effort = -1; 
                gripperGoal.command = gripperCommand;
                gripperClient.sendGoal(gripperGoal);
                bool gripSuccess=  gripperClient.waitForResult();
                if (gripSuccess){ROS_INFO_NAMED("graspDemo","Gripper Oppened Succesfully");}
                else{ ROS_ERROR("Gripper Failed To open please override Manually");}

                std::cin.ignore();
                control_msgs::FollowJointTrajectoryGoal trajGoal;
                trajGoal.trajectory = trajectory.joint_trajectory;
                armClient.sendGoal(trajGoal);
                while (num_tries<5&&!execute_grasp){
                        execute_grasp =  armClient.waitForResult();
                        num_tries+=1;
                }
                if (execute_grasp){ROS_INFO("ACTION WAS EXECUTED SUCCESFULY!!!!!!");}
                else{ ROS_ERROR("Execution failed ");}

                std::cout << "To close the  Gripper Press Enter" << std::endl;
                std::cin.ignore();

                ROS_INFO("Closing  gripper ..");
                gripperCommand.position = 0.02; 
                gripperCommand.max_effort = -1; 
                gripperGoal.command = gripperCommand;
                gripperClient.sendGoal(gripperGoal);
                //gripSuccess = gripperClient.waitForResult();
                if (gripSuccess){ROS_INFO_NAMED("graspDemo","ACTION WAS EXECUTED SUCCESFULY!!!!!!");}
                else{ROS_ERROR_NAMED("graspDemo","Gripper Failed To close please override Manually");}
            
            }
        }

       ROS_INFO_NAMED("graspDemo","This attemp reached termination ...");
       char next_action;
       std::cout << "To Execute another Grasp Enter Y" << std::endl;
       std::cin>> next_action;
       if (next_action == 'Y'){ attemp_grasp = true;}
       else { attemp_grasp = false;}
    }
  
}
