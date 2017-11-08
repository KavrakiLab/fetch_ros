/* Author: Chamzas Constantinos */
// ROS stuff
#include <tf2_ros/transform_listener.h>

#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
void printTF(tf::Transform tra)
{
    tf::Quaternion quat;
    tf::Point orig;
    quat = tra.getRotation();
    orig = tra.getOrigin();
    std::cout << "Quat:[X,Y,Z,W]=[" << quat.getX() << ',' << quat.getY() << "," << quat.getZ()
              << "," << quat.getW() << ']' << std::endl;
    std::cout << "Orig:[X,Y,Z]=[" << orig.getX() << ',' << orig.getY() << "," << orig.getZ() << ']'
              << std::endl;
}

class getGrasps
{
public:
    getGrasps(ros::NodeHandle& n) : n_(n), tfListener_(tfBuffer_)
    {
        sub_ = n_.subscribe("/detect_grasps/clustered_grasps", 3, &getGrasps::callback, this);
        graspBest_.score.data = 0;
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
        // Needs to be validated again
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
            ROS_ERROR("getGrasps::lookup:%s,%s):%s", base.c_str(), target.c_str(), ex.what());
        }
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
                ROS_ERROR("Grasp Updated");
                graspTF_ = graspCurrTF;
                printTF(graspTF_);
                graspBest_ = graspCurr;
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
        graspMsgPose.position.z = 0.88;

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

    getGrasps* grasps = new getGrasps(n);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual: world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Raw pointers are frequently used to refer to the planning kgroup for
    // improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setPoseReferenceFrame("/base_link");
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("/base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "setPoseReferenceFrame %s",
                   move_group.getPoseReferenceFrame().c_str());

    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // waiting for a grasp to be received
    std::cout << "Waiting For Grasps to Be Received To Continue Press Enter" << std::endl;
    std::cin.ignore();

    geometry_msgs::Pose objGrasp = grasps->getGraspPoseMsg();
    geometry_msgs::Pose preGrasp = grasps->getGraspPoseMsg(0.3);
    geometry_msgs::Pose finGrasp = grasps->getGraspPoseMsg(0.1);

    visual_tools.publishAxisLabeled(objGrasp, "OnObject");
    visual_tools.publishAxisLabeled(preGrasp, "preGrasp");
    visual_tools.publishAxisLabeled(finGrasp, "Grasp");
    visual_tools.trigger();
    std::cout << "To plan the path press Enter" << std::endl;
    std::cin.ignore();

    // Planning to a Pose goal
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    move_group.setPoseTarget(preGrasp);
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = move_group.plan(my_plan);

    std::cout << "To execute the path press Enter" << std::endl;
    std::cin.ignore();
    move_group.move();

    std::cout << "To Open the Gripper press Enter" << std::endl;
    std::cin.ignore();
    moveit::planning_interface::MoveGroupInterface move_group_gripper(PLANNING_GROUP_GRIPPER);
    move_group_gripper.setJointValueTarget("r_gripper_finger_joint", 0.05);
    move_group_gripper.setJointValueTarget("l_gripper_finger_joint", 0.05);
    success = move_group_gripper.plan(my_plan);
    move_group_gripper.move();
    std::cout << "To Close the Gripper press Enter" << std::endl;
    std::cin.ignore();
    move_group_gripper.setJointValueTarget("r_gripper_finger_joint", 0.02);
    move_group_gripper.setJointValueTarget("l_gripper_finger_joint", 0.02);
    success = move_group_gripper.plan(my_plan);
    move_group_gripper.move();
    std::cin.ignore();
}
