
// ***** MoveRs (7-DOF) ACTION SERVER ***** //

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_rs.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi/180.0;

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

// Declaration of GLOBAL VARIABLE --> ROBOT / END-EFFECTOR PARAMETER:
std::string my_param = "none";

class ros2_RobotTrigger : public rclcpp::Node
{
public:
    ros2_RobotTrigger() : Node("ros2_RobotTrigger_PARAM") 
    {
        this->declare_parameter("ROB_PARAM", "null");
        my_param = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", my_param.c_str());
    }
private:
};

class ActionServer : public rclcpp::Node
{
public:
    using MoveRs = ros2_data::action::MoveRs;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveRs>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MoveRs_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<MoveRs>(
            this,
            "/MoveRs",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<MoveRs>::SharedPtr action_server_;
    
    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveRs::Goal> goal)
    {
        auto joint = goal->joint;
        double value = goal->value;
        RCLCPP_INFO(get_logger(), "Received a MoveRs request, with Joint+Value -> (%s -> %.2f)", joint.c_str(), value);
        //(void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept and execute the goal received.
    }

    // No idea about what this function does:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        
    }

    // Function that cancels the goal request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

        // We call the -> void moveit::planning_interface::MoveGroupInterface::stop(void) method,
        // which stops any trajectory execution, if one is active.
        move_group_interface.stop();

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting MoveRs motion...");
        
        rclcpp::Rate loop_rate(0.5);
        
        // Obtain input value (goal -- Joint+Value):
        const auto goal = goal_handle->get_goal();
        auto joint = goal->joint;
        double value = goal->value;
        
        // Initialise empty jx variables:
        double j1, j2, j3, j4, j5, j6, j7, j8;
        j1 = j2 = j3 = j4 = j5 = j6 = j7 = j8 = 0.0;
        
        // Obtain JOINT SPEED and apply it into MoveIt!2:
        auto SPEED = goal->speed;
        move_group_interface.setPlannerId("PTP");
        move_group_interface.setMaxVelocityScalingFactor(SPEED);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        
        // FEEDBACK?
        // No feedback needed for MoveRs Action Calls.
        // No loop needed for MoveRs Action Calls.

        // Declare RESULT:
        auto result = std::make_shared<MoveRs::Result>();

        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(my_param);

        // Obtain current JOINT VALUES:
        auto current_JointValues = move_group_interface.getCurrentJointValues();
        j1 = current_JointValues[0] * (1/k);
        j2 = current_JointValues[1];
        j3 = current_JointValues[2];
        j4 = current_JointValues[3];
        j5 = current_JointValues[4];
        j6 = current_JointValues[5];
        j7 = current_JointValues[6];
        j8 = current_JointValues[7] * (1/k);

        RCLCPP_INFO(this->get_logger(), "Current JOINT VALUES before MoveRs are -> (j1 = %.2f, j2 = %.2f, j3 = %.2f, j4 = %.2f, j5 = %.2f, j6 = %.2f, j7 = %.2f, j8 = %.2f)", j1, j2, j3, j4, j5, j6, j7,j8);
        
        double j1UL, j1LL, j2UL, j2LL, j3UL, j3LL, j4UL, j4LL, j5UL, j5LL, j6UL, j6LL, j7UL, j7LL, j8UL, j8LL = 0.0;

        // ***** JOINT VALUES (MAX/MIN) ***** //
        if (my_param == "arm_towercrane"){
            j1UL = 200;
            j1LL = -160;
            j2UL = 30;
            j2LL = 0;
            j3UL = 5;
            j3LL = 0;
            j4UL = 5;
            j4LL = 0;
            j5UL = 5;
            j5LL = 0;
            j6UL = 5;
            j6LL = 0;
            j7UL = 5;
            j7LL = 0;
            j8UL = 180;
            j8LL = -180;
    
        } else if (my_param == "iiwa_arm"){
            j1UL = 170;
            j1LL = -170;
            j2UL = 120;
            j2LL = -120;
            j3UL = 170;
            j3LL = -170;
            j4UL = 120;
            j4LL = -120;
            j5UL = 170;
            j5LL = -170;
            j6UL = 120;
            j6LL = -120;
            j7UL = 175;
            j7LL = -175;
        };
        
        // Check if INPUT JOINT VALUES are within the JOINT LIMIT VALUES:
        bool LimitCheck = false;
        auto InputJoint = "Valid";

        if (joint == "joint_1"){
            j1 = j1 + value;
            if (j1 <= j1UL && j1 >= j1LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_2"){
            j2 = j2 + value;
            if (j2 <= j2UL && j2 >= j2LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_3"){
            j3 = j3 + value;
            if (j3 <= j3UL && j3 >= j3LL&& LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_4"){
            j4 = j4 + value;
            if (j4 <= j4UL && j4 >= j4LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_5"){
            j5 = j5 + value;
            if (j5 <= j5UL&& j5 >= j5LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_6"){
            j6 = j6 + value;
            if (j6 <= j6UL && j6 >= j6LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_7"){
            j7 = j7 + value;
            if (j7 <= j7UL && j7 >= j7LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint_8"){
            j8 = j8 + value;
            if (j8 <= j8UL && j8 >= j8LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        }  
        else {
            LimitCheck = true;
            InputJoint = "NotValid";
        }
            
        // EXECUTE IF -> JointValues are within the limits:
        if (LimitCheck == false){
            
            // JOINT-goal planning:
            moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            joint_group_positions[0] = j1 * k;
            joint_group_positions[1] = j2;
            joint_group_positions[2] = j3;
            joint_group_positions[3] = j4;
            joint_group_positions[4] = j5;
            joint_group_positions[5] = j6; 
            joint_group_positions[6] = j7;
            joint_group_positions[7] = j8 * k;
            move_group_interface.setJointValueTarget(joint_group_positions);
            move_group_interface.setPlannerId("PTP");


            // Plan, execute and inform (with feedback):
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            
            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(success) {

                RCLCPP_INFO(this->get_logger(), "%s - MoveRs: Planning successful!", my_param.c_str());
                move_group_interface.move();
                
                // Do if GOAL CANCELLED:
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                    result->result = "MoveRs:CANCELED";
                    goal_handle->canceled(result);
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s - MoveRs: Movement executed!", my_param.c_str());
                    result->result = "MoveRs:SUCCESS";
                    goal_handle->succeed(result);
                }

            } else {
                RCLCPP_INFO(this->get_logger(), "%s - MoveRs: Planning failed!", my_param.c_str());
                result->result = "MoveRs:FAILED";
                goal_handle->succeed(result);
            }    

        } else if (LimitCheck == true && InputJoint == "Valid") {

            RCLCPP_INFO(this->get_logger(), "%s - MoveRs: Planning failed, JOINT LIMITS exceeded!", my_param.c_str());
            result->result = "MoveRs:FAILED";
            goal_handle->succeed(result);

        } else {

            RCLCPP_INFO(this->get_logger(), "%s - MoveRs: Planning failed, JOINT NAME NOT VALID!", my_param.c_str());
            result->result = "MoveRs:FAILED";
            goal_handle->succeed(result);

        }

    }

};

int main(int argc, char ** argv)
{
  // Initialise MAIN NODE:
  rclcpp::init(argc, argv);

  // Obtain ros2_RobotTrigger parameter:
  auto node_PARAM = std::make_shared<ros2_RobotTrigger>();
  rclcpp::spin_some(node_PARAM);

  // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
  auto name = "_MoveRs_interface";
  auto node2name = my_param + name;
  auto const node2 = std::make_shared<rclcpp::Node>(
      node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the Move Group Interface:
  using moveit::planning_interface::MoveGroupInterface;
  move_group_interface = MoveGroupInterface(node2, my_param);
  // Create the MoveIt PlanningScene Interface:
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // Declare and spin ACTION SERVER:
  auto action_server = std::make_shared<ActionServer>();
  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}