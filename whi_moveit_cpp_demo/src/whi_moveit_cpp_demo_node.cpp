/******************************************************************
node of MoveItCpp demo

Features:
- MoveItCpp interface demo
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-05-31: Initial version
2022-xx-xx: xxx
******************************************************************/
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/src/Geometry/Transform.h>

int main(int argc, char** argv)
{
    /// node version and copyright announcement
    std::cout << "\nWHI MoveIt MoveItCpp demo VERSION 00.01" << std::endl;
    std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

    ros::init(argc, argv, "moveit_cpp_demo");
    ros::NodeHandle nodeHandle("/moveit_cpp_tutorial");

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand
    ros::AsyncSpinner spinner(4);
    spinner.start();

    /// get config params
    std::string paramPlanningGroup;
    nodeHandle.param("/moveit_cpp_demo/planning_group", paramPlanningGroup, std::string("whi_arm"));
    std::string paramVisualLink;
    nodeHandle.param("/moveit_cpp_demo/visual_link", paramVisualLink, std::string("whi_link0"));
    double paramTitleHeight = 0.0;
    nodeHandle.param("/moveit_cpp_demo/title_height", paramTitleHeight, 0.7);
    // plan01
    std::vector<double> paramGoal01;
    nodeHandle.getParam("/moveit_cpp_demo/plan01/goal_pose", paramGoal01);
    std::string paramGoalFrame;
    nodeHandle.param("/moveit_cpp_demo/plan01/goal_frame", paramGoalFrame, std::string("whi_link0"));
    std::string paramGoalLink;
    nodeHandle.param("/moveit_cpp_demo/plan01/goal_link", paramGoalLink, std::string("whi_link7"));

    /// setup
    //
    // otherwise robot with zeros joint_states
    ros::Duration(1.0).sleep();

    namespace moveit_cpp = moveit::planning_interface;
    auto moveitCppPtr = std::make_shared<moveit_cpp::MoveItCpp>(nodeHandle);
    moveitCppPtr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planningComponents = std::make_shared<moveit_cpp::PlanningComponent>(paramPlanningGroup, moveitCppPtr);
    auto robotModelPtr = moveitCppPtr->getRobotModel();
    auto robotStartState = planningComponents->getStartState();
    auto jointModelGroupPtr = robotModelPtr->getJointModelGroup(paramPlanningGroup);

    // visualization
    //
    // MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools(paramVisualLink, rvt::RVIZ_MARKER_TOPIC,
        moveitCppPtr->getPlanningSceneMonitor());
    visualTools.deleteAllMarkers();
    visualTools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = paramTitleHeight;
    visualTools.publishText(text_pose, "MoveItCpp Demo", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    /// start the demo
    //
    visualTools.prompt("Press 'next' to start the demo");

    /// planning with MoveItCpp
    //
    // there are multiple ways to set the start and the goal states of the plan
    // they are illustrated in the following plan examples
    //
    // the start state of the plan can be set from the current state of the robot
    planningComponents->setStartStateToCurrentState();

    /// plan 01
    //
    // the first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
    geometry_msgs::PoseStamped goal01;
    goal01.header.frame_id = paramGoalFrame;
    goal01.pose.orientation.w = 1.0;
    goal01.pose.position.x = paramGoal01[0];
    goal01.pose.position.y = paramGoal01[1];
    goal01.pose.position.z = paramGoal01[2];
    planningComponents->setGoal(goal01, paramGoalLink);

    // then call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    auto planSolution01 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution01)
    {
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartState->getGlobalLinkTransform("panda_link8"), "start_pose");
        visualTools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal01.pose, "target_pose");
        visualTools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution01.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 02");

    /// plan 02
    //
    // it will set the current state of the plan using moveit::core::RobotState
    auto startState = *(moveitCppPtr->getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.55;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.6;

    startState.setFromIK(jointModelGroupPtr, start_pose);

    planningComponents->setStartState(startState);

    // reuse the old goal that we had and plan to it
    auto planSolution02 = planningComponents->plan();
    if (planSolution02)
    {
        moveit::core::RobotState robotState(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(planSolution02.start_state, robotState);

        visualTools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform("panda_link8"), "start_pose");
        visualTools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(goal01.pose, "target_pose");
        visualTools.publishTrajectoryLine(planSolution02.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 03");

    /// plan 03
    //
    // the goal of the plan can be set by using moveit::core::RobotState
    auto targetState = *robotStartState;
    geometry_msgs::Pose goal03;
    goal03.orientation.w = 1.0;
    goal03.position.x = 0.55;
    goal03.position.y = -0.05;
    goal03.position.z = 0.8;

    targetState.setFromIK(jointModelGroupPtr, goal03);

    planningComponents->setGoal(targetState);

    // reuse the old start that we had and plan from it
    auto plan_solution3 = planningComponents->plan();
    if (plan_solution3)
    {
        moveit::core::RobotState robot_state(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

        visualTools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
        visualTools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(goal03, "target_pose");
        visualTools.publishTrajectoryLine(plan_solution3.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 04");

    /// plan 04
    //
    // the start state of the plan can be set by the current state of the robot
    // the goal of the plan can be set by using the name of a group states
    // the planning group called "ready" is the prerequisite

    // set the goal state of the plan from a named robot state
    planningComponents->setGoal("ready");

    // reuse the old start that we had and plan from it
    auto planSolution04 = planningComponents->plan();
    if (planSolution04)
    {
        moveit::core::RobotState robotState(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(planSolution04.start_state, robotState);

        visualTools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform("panda_link8"), "start_pose");
        visualTools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotStartState->getGlobalLinkTransform("panda_link8"), "target_pose");
        visualTools.publishTrajectoryLine(planSolution04.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to exit the demo");

    ros::shutdown();
    return 0;
}
