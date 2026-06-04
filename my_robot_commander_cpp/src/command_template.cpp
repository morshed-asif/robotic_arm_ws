#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "example_interfaces/msg/bool.hpp"
#include "my_robot_interfaces/msg/pose_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        // Move group interfaces
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        // Configuration
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // Subscribers
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10,
            std::bind(&Commander::openGripperCallback, this, std::placeholders::_1));

        pose_command_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 10,
            std::bind(&Commander::poseCommandCallback, this, std::placeholders::_1));
    }

    // ── Public methods ──────────────────────────────────────────

    void goToNamedTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z,
                        double roll, double pitch, double yaw,
                        bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (!cartesian_path) {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        } else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            
            double fraction = arm_->computeCartesianPath(
                waypoints,   // target poses
                0.01,        // eef_step
                0.0,         // jump_threshold
                trajectory,  // output trajectory
                true,        // avoid_collisions
                nullptr      // error_code pointer
            );
            if (fraction == 1.0) {
                arm_->execute(trajectory);
            }
        }
    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }

private:
    // ── Private methods ─────────────────────────────────────────

    void planAndExecute(const std::shared_ptr<MoveGroupInterface>& interface)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            interface->execute(plan);
        }
    }

    void openGripperCallback(const Bool::SharedPtr msg)
    {
        if (msg->data) openGripper();
        else closeGripper();
    }

    void poseCommandCallback(const PoseCmd::SharedPtr msg)
    {
        goToPoseTarget(msg->x, msg->y, msg->z,
                       msg->roll, msg->pitch, msg->yaw,
                       msg->cartesian_path);
    }

    // ── Members ─────────────────────────────────────────────────
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_command_sub_;
};

// ── main ────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = std::make_shared<Commander>(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
