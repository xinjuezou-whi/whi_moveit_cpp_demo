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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/src/Geometry/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include <thread>

int main(int argc, char** argv)
{
    /// node version and copyright announcement
    std::cout << "\nWHI MoveIt MoveItCpp demo VERSION 00.05" << std::endl;
    std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

    ros::init(argc, argv, "moveit_cpp_demo");
    ros::NodeHandle nodeHandle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand
    ros::AsyncSpinner spinner(4);
    spinner.start();

    /// get config params
    std::string paramPlanningGroup;
    nodeHandle.param("/moveit_cpp_demo/planning_group", paramPlanningGroup, std::string("whi_arm"));
    std::string paramVisualFrame;
    nodeHandle.param("/moveit_cpp_demo/visual_frame", paramVisualFrame, std::string("whi_link0"));
    std::string paramVisualLink;
    nodeHandle.param("/moveit_cpp_demo/visual_link", paramVisualLink, std::string("whi_link7"));
    double paramTitleHeight = 0.0;
    nodeHandle.param("/moveit_cpp_demo/title_height", paramTitleHeight, 0.7);
    // plan01
    std::vector<double> paramGoal01;
    nodeHandle.getParam("/moveit_cpp_demo/plan01/goal_pose", paramGoal01);
    std::string paramGoalFrame;
    nodeHandle.param("/moveit_cpp_demo/plan01/goal_frame", paramGoalFrame, std::string("whi_link0"));
    std::string paramGoalLink;
    nodeHandle.param("/moveit_cpp_demo/plan01/goal_link", paramGoalLink, std::string("whi_link7"));
    // plan02
    std::vector<int> paramJointIndex;
    nodeHandle.getParam("/moveit_cpp_demo/plan02/joint_index", paramJointIndex);
    std::vector<double> paramJointDelta;
    nodeHandle.getParam("/moveit_cpp_demo/plan02/joint_goal", paramJointDelta);
    double paramVelScalingFactor;
    nodeHandle.getParam("/moveit_cpp_demo/plan02/velocity_scaling_factor", paramVelScalingFactor);
    double paramAccScalingFactor;
    nodeHandle.getParam("/moveit_cpp_demo/plan02/acc_scaling_factor", paramAccScalingFactor);
    // plan03
    double paramIkTimeout03 = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan03/ik_timeout", paramIkTimeout03, 0.5);
    std::vector<double> paramStartPose03;
    nodeHandle.getParam("/moveit_cpp_demo/plan03/start_pose", paramStartPose03);
    // plan04
    std::vector<double> paramGoal04;
    nodeHandle.getParam("/moveit_cpp_demo/plan04/goal_pose", paramGoal04);
    // plan05
    std::string paramStateGroup;
    nodeHandle.param("/moveit_cpp_demo/plan05/state_group", paramStateGroup, std::string("ready"));
    // plan06
    double paramIkTimeout06 = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan06/ik_timeout", paramIkTimeout06, 0.5);
    std::vector<double> paramStartPose06;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/start_pose", paramStartPose06);
    std::vector<std::string> paramPoseIndex01;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/pose_index_01", paramPoseIndex01);
    std::vector<double> paramPoseDelta01;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/pose_delta_01", paramPoseDelta01);
    std::vector<std::string> paramPoseIndex02;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/pose_index_02", paramPoseIndex02);
    std::vector<double> paramPoseDelta02;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/pose_delta_02", paramPoseDelta02);
    std::vector<std::string> paramPoseIndex03;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/pose_index_03", paramPoseIndex03);
    std::vector<double> paramPoseDelta03;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/pose_delta_03", paramPoseDelta03);
    double paramJumpThreshold = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan06/jump_threshold", paramJumpThreshold, 0.0);
    double paramEndEffectorStep = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan06/end_effector_step", paramEndEffectorStep, 0.01);
    // plan07
    std::vector<double> paramGoal07;
    nodeHandle.getParam("/moveit_cpp_demo/plan07/goal_pose", paramGoal07);
    std::vector<double> paramBoxSize;
    nodeHandle.getParam("/moveit_cpp_demo/plan07/block_box_size", paramBoxSize);
    std::vector<double> paramBoxPose;
    nodeHandle.getParam("/moveit_cpp_demo/plan07/block_box_pose", paramBoxPose);
    double paramCylinderRadius = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan07/grab_cylinder_radius", paramCylinderRadius, 0.02);
    double paramCylinderHeight = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan07/grab_cylinder_height", paramCylinderHeight, 0.1);
    std::vector<std::string> paramCylinderPoseIndex;
    nodeHandle.getParam("/moveit_cpp_demo/plan07/grap_cylinder_pose_index", paramCylinderPoseIndex);
    std::vector<double> paramCylinderPose;
    nodeHandle.getParam("/moveit_cpp_demo/plan07/grap_cylinder_pose", paramCylinderPose);

    /// setup
    //
    // otherwise robot with zeros joint_states
    ros::Duration(1.0).sleep();

    namespace moveit_cpp = moveit::planning_interface;
    auto moveitCppPtr = std::make_shared<moveit_cpp::MoveItCpp>(nodeHandle);
    moveitCppPtr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planningComponents = std::make_shared<moveit_cpp::PlanningComponent>(paramPlanningGroup, moveitCppPtr);
    auto robotModelPtr = moveitCppPtr->getRobotModel();
    auto robotStartStatePtr = planningComponents->getStartState();
    auto jointModelGroupPtr = robotModelPtr->getJointModelGroup(paramPlanningGroup);

    geometry_msgs::Pose robotStartPose;//= tf2::toMsg(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink)); // failed to compile why?
    tf::poseEigenToMsg(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink), robotStartPose);
    int dof = jointModelGroupPtr->getVariableNames().size();

    // visualization
    //
    // MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools(paramVisualFrame, rvt::RVIZ_MARKER_TOPIC,
        moveitCppPtr->getPlanningSceneMonitor());
    visualTools.deleteAllMarkers();
    visualTools.loadRemoteControl();

    Eigen::Isometry3d textPose = Eigen::Isometry3d::Identity();
    textPose.translation().z() = paramTitleHeight;
    visualTools.publishText(textPose, "MoveItCpp Demo", rvt::WHITE, rvt::XLARGE);
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
    if (dof > 6)
    {
        // DOF 7
        goal01.pose.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal01.pose.orientation = robotStartPose.orientation;
    }
    goal01.pose.position.x = paramGoal01[0];
    goal01.pose.position.y = paramGoal01[1];
    goal01.pose.position.z = paramGoal01[2];
    planningComponents->setGoal(goal01, paramGoalLink);

    // call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    auto planSolution01 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution01)
    {
        visualTools.publishText(textPose, "single goal", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal01.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution01.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 02");

    /// Plan02: planning to a joint-space goal
    //
    // get the current set of joint values for the group
    std::vector<double> jointGroupPositions;
    robotStartStatePtr->copyJointGroupPositions(jointModelGroupPtr, jointGroupPositions);
    // modify the position of the joints, plan to the new joint space goal and visualize the plan
    for (std::size_t i = 0; i < paramJointIndex.size(); ++i)
    {
        jointGroupPositions[paramJointIndex[i]] = paramJointDelta[i];
    }
    auto goalState02 = *(moveitCppPtr->getCurrentState());
    goalState02.setJointGroupPositions(jointModelGroupPtr, jointGroupPositions);
    // set goal with RobotState
    planningComponents->setGoal(goalState02);
    auto planSolution02 = planningComponents->plan();
    if (planSolution02)
    {
        visualTools.publishText(textPose, "joint space goal", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink), "start_pose");
        moveit::core::RobotState robotTrajLastState = planSolution02.trajectory->getLastWayPoint();
        visualTools.publishAxisLabeled(robotTrajLastState.getGlobalLinkTransform(paramVisualLink), "target_pose");
        // visualize the trajectory in rviz
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
    // the current state of the plan can be set by using moveit::core::RobotState
    auto startState03 = *(moveitCppPtr->getCurrentState());
    geometry_msgs::Pose startPose03;
    startPose03.orientation.w = 1.0;
    if (dof > 6)
    {
        // DOF 7
        startPose03.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        startPose03.orientation = robotStartPose.orientation;
    }
    startPose03.position.x = paramStartPose03[0];
    startPose03.position.y = paramStartPose03[1];
    startPose03.position.z = paramStartPose03[2];

    if (startState03.setFromIK(jointModelGroupPtr, startPose03, paramIkTimeout03))
    {
        planningComponents->setStartState(startState03);

        // reuse the goal of plan02 that we had and plan to it
        auto planSolution03 = planningComponents->plan();
        if (planSolution03)
        {
            moveit::core::RobotState robotState(robotModelPtr);
            moveit::core::robotStateMsgToRobotState(planSolution03.start_state, robotState);

            visualTools.publishText(textPose, "single goal with pre-set start state", rvt::WHITE, rvt::XLARGE);
            visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform(paramVisualLink), "start_pose");
            if (planSolution02)
            {
                moveit::core::RobotState robotTrajLastState = planSolution02.trajectory->getLastWayPoint();
                visualTools.publishAxisLabeled(robotTrajLastState.getGlobalLinkTransform(paramVisualLink), "target_pose");
            }
            visualTools.publishTrajectoryLine(planSolution03.trajectory, jointModelGroupPtr);
            visualTools.trigger();

            // uncomment if you want to execute the plan
            //planningComponents->execute();
        }
    }
    else
    {
        visualTools.publishText(textPose, "failed to get the IK from pre-set start state", rvt::WHITE, rvt::XLARGE);
        if (planSolution02)
        {
            moveit::core::RobotState robotTrajLastState = planSolution02.trajectory->getLastWayPoint();
            visualTools.publishAxisLabeled(robotTrajLastState.getGlobalLinkTransform(paramVisualLink), "target_pose");
        }
        visualTools.trigger();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 04");

    /// plan 04
    //
    // the goal of the plan can be set by using moveit::core::RobotState
    auto targetState = *robotStartStatePtr;
    geometry_msgs::Pose goal04;
    if (dof > 6)
    {
        // DOF 7
        goal04.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal04.orientation = robotStartPose.orientation;
    }
    goal04.position.x = paramGoal04[0];
    goal04.position.y = paramGoal04[1];
    goal04.position.z = paramGoal04[2];

    targetState.setFromIK(jointModelGroupPtr, goal04);

    planningComponents->setGoal(targetState);

    // reuse the previous start state that we had and plan from it
    auto planSolution04 = planningComponents->plan();
    if (planSolution04)
    {
        moveit::core::RobotState robotState(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(planSolution04.start_state, robotState);

        visualTools.publishText(textPose, "single goal set by target state", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform(paramVisualLink), "start_pose");
        visualTools.publishAxisLabeled(goal04, "target_pose");
        visualTools.publishTrajectoryLine(planSolution04.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 05");

    /// plan 05
    //
    // the start state of the plan can be set by the current state of the robot
    // the goal of the plan can be set by using the name of a group states
    // the planning group called "ready" is the prerequisite

    // set the goal state of the plan from a named robot state
    planningComponents->setGoal(paramStateGroup);

    // reuse the previous start state that we had and plan from it
    auto planSolution05 = planningComponents->plan();
    if (planSolution05)
    {
        moveit::core::RobotState robotState(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(planSolution05.start_state, robotState);

        visualTools.publishText(textPose, "goal from state group", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform(paramVisualLink), "start_pose");
        moveit::core::RobotState robotTrajLastState = planSolution05.trajectory->getLastWayPoint();
        visualTools.publishAxisLabeled(robotTrajLastState.getGlobalLinkTransform(paramVisualLink), "target_pose");
        visualTools.publishTrajectoryLine(planSolution05.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 06");

    // plan06
    // 
    // restore the start state
    planningComponents->setStartState(*robotStartStatePtr);
    // set the state to the one which satisfies the constraints of Cartesian
    auto startState06 = *(moveitCppPtr->getCurrentState());
    geometry_msgs::Pose startPose06;
    if (dof > 6)
    {
        // DOF 7
        startPose06.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        startPose06.orientation = robotStartPose.orientation;
    }
    startPose06.position.x = paramStartPose06[0];
    startPose06.position.y = paramStartPose06[1];
    startPose06.position.z = paramStartPose06[2];
    if (startState06.setFromIK(jointModelGroupPtr, startPose06, paramIkTimeout06))
    {
        planningComponents->setStartState(startState06);

        // organize waypoints
        EigenSTL::vector_Isometry3d waypoints;
        Eigen::Isometry3d eigenPose;
        tf::poseMsgToEigen(startPose06, eigenPose);
        waypoints.push_back(eigenPose);

        geometry_msgs::Pose wayPose = startPose06;
        // up and back
        for (std::size_t i = 0; i < paramPoseIndex01.size(); ++i)
        {
            if (paramPoseIndex01[i] == "x")
            {
                wayPose.position.x += paramPoseDelta01[i];
            }
            else if (paramPoseIndex01[i] == "y")
            {
                wayPose.position.y += paramPoseDelta01[i];
            }
            else
            {
                wayPose.position.z += paramPoseDelta01[i];
            }
        }
        tf::poseMsgToEigen(wayPose, eigenPose);
        waypoints.push_back(eigenPose);
        // left
        for (std::size_t i = 0; i < paramPoseIndex02.size(); ++i)
        {
            if (paramPoseIndex02[i] == "x")
            {
                wayPose.position.x += paramPoseDelta02[i];
            }
            else if (paramPoseIndex02[i] == "y")
            {
                wayPose.position.y += paramPoseDelta02[i];
            }
            else
            {
                wayPose.position.z += paramPoseDelta02[i];
            }
        }
        tf::poseMsgToEigen(wayPose, eigenPose);
        waypoints.push_back(eigenPose);
        // down and right
        for (std::size_t i = 0; i < paramPoseIndex03.size(); ++i)
        {
            if (paramPoseIndex03[i] == "x")
            {
                wayPose.position.x += paramPoseDelta03[i];
            }
            else if (paramPoseIndex03[i] == "y")
            {
                wayPose.position.y += paramPoseDelta03[i];
            }
            else
            {
                wayPose.position.z += paramPoseDelta03[i];
            }
        }
        tf::poseMsgToEigen(wayPose, eigenPose);
        waypoints.push_back(eigenPose);

        std::vector<moveit::core::RobotStatePtr> resTraj;
        double fraction = startState06.computeCartesianPath(jointModelGroupPtr, resTraj, jointModelGroupPtr->getLinkModel(paramVisualLink), waypoints,
            true, paramEndEffectorStep, paramJumpThreshold);
        ROS_INFO_NAMED("demo", "visualizing plan 06 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
        
        /// to visualize the Cartesian path
        // get the robot_trajectory::RobotTrajectory from RobotStatePtr
        robot_trajectory::RobotTrajectory traj(robotModelPtr, paramPlanningGroup);
        for (const moveit::core::RobotStatePtr& trajState : resTraj)
        {
            traj.addSuffixWayPoint(trajState, 0.0);
        }
        // get the moveit_msgs::RobotTrajectory from robot_trajectory::RobotTrajectory
        moveit_msgs::RobotTrajectory msgTraj;
        traj.getRobotTrajectoryMsg(msgTraj);
        if (traj.getWayPointCount() > 0)
        {
            moveit_msgs::DisplayTrajectory dispTraj;
            dispTraj.model_id = robotModelPtr->getName();
            dispTraj.trajectory.resize(1, msgTraj);
            moveit::core::robotStateToRobotStateMsg(traj.getFirstWayPoint(), dispTraj.trajectory_start);
            auto displayPathPub = nodeHandle.advertise<moveit_msgs::DisplayTrajectory>(
                "/whi_moveit_cpp_demo/ompl/display_planned_path"/*planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC*/, 10, true);
            displayPathPub.publish(dispTraj);
        }

        visualTools.publishText(textPose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        {
            visualTools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        }
        // still have no idea how to leverage visualTools to publish the Cartesian path???
        //visualTools.publishTrajectoryPath(resTraj, jointModelGroupPtr);
        visualTools.trigger();
    }
    else
    {
        visualTools.publishText(textPose, "failed to get the IK from pre-set start state", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(startPose06, "target_pose");
        visualTools.trigger();
    }
    
    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 07");

    /// plan 07
    //
    // restore the start state
    planningComponents->setStartState(*robotStartStatePtr);
    // first plan a clear goal
    geometry_msgs::PoseStamped goal07;
    goal07.header.frame_id = paramGoalFrame;
    if (dof > 6)
    {
        // DOF 7
        goal07.pose.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal07.pose.orientation = robotStartPose.orientation;
    }
    goal07.pose.position.x = paramGoal07[0];
    goal07.pose.position.y = paramGoal07[1];
    goal07.pose.position.z = paramGoal07[2];
    planningComponents->setGoal(goal07, paramGoalLink);

    // call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    auto planSolution07 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution07)
    {
        visualTools.publishText(textPose, "clear goal", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal07.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution07.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with next step of plan 07");

    // define a collision box to add to the world
    shape_msgs::SolidPrimitive boxPrimitive;
    boxPrimitive.type = boxPrimitive.BOX;
    boxPrimitive.dimensions.resize(3);
    boxPrimitive.dimensions[boxPrimitive.BOX_X] = paramBoxSize[0];
    boxPrimitive.dimensions[boxPrimitive.BOX_Y] = paramBoxSize[1];
    boxPrimitive.dimensions[boxPrimitive.BOX_Z] = paramBoxSize[2];
    // define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose boxPose;
    boxPose.orientation.w = 1.0;
    boxPose.position.x = paramBoxPose[0];
    boxPose.position.y = paramBoxPose[1];
    boxPose.position.z = paramBoxPose[2];
    // define a collision object ROS message for the robot to avoid
    moveit_msgs::CollisionObject collisionObject;
    // frame_id decides the coord frame the object belongs to
    collisionObject.header.frame_id = robotModelPtr->getRootLinkName();
    // the id of the object is used to identify it
    collisionObject.id = "collision box";
    collisionObject.primitives.push_back(boxPrimitive);
    collisionObject.primitive_poses.push_back(boxPose);
    collisionObject.operation = collisionObject.ADD;
    {
        // to access the PlanningSceneMonitor's underlying PlanningScene,
        // use the provided LockedPlanningSceneRW and LockedPlanningSceneRO classes
        // lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW planningScene(moveitCppPtr->getPlanningSceneMonitor());
        planningScene->processCollisionObjectMsg(collisionObject);
        // unlock PlanningScene
    }

    // re-call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    planSolution07 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution07)
    {
        visualTools.publishText(textPose, "obstacle goal", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal07.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution07.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with next step of plan 07");

    // define a cylinder to attach to the arm
    shape_msgs::SolidPrimitive cylinderPrimitive;
    cylinderPrimitive.type = cylinderPrimitive.CYLINDER;
    cylinderPrimitive.dimensions.resize(2);
    cylinderPrimitive.dimensions[cylinderPrimitive.CYLINDER_HEIGHT] = paramCylinderHeight;
    cylinderPrimitive.dimensions[cylinderPrimitive.CYLINDER_RADIUS] = paramCylinderRadius;
    // define a pose for the cylinder (specified relative to frame_id)
    geometry_msgs::Pose grabPose;
    if (dof > 6)
    {
        // DOF 7
        grabPose.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        grabPose.orientation = robotStartPose.orientation;
    }
    for (std::size_t i = 0; i < paramCylinderPoseIndex.size(); ++i)
    {
        if (paramCylinderPoseIndex[i] == "x")
        {
            grabPose.position.x = paramCylinderPose[i];
        }
        else if (paramCylinderPoseIndex[i] == "y")
        {
            grabPose.position.y = paramCylinderPose[i];
        }
        else
        {
            grabPose.position.z = paramCylinderPose[i];
        }
    }
    // define a collision object ROS message for the robot to avoid
    moveit_msgs::AttachedCollisionObject attachedObject;
    //attachedObject.object = object2Attach;
    attachedObject.link_name = jointModelGroupPtr->getLinkModelNames().back();
    // the header must contain a valid TF frame
    attachedObject.object.header.frame_id = jointModelGroupPtr->getLinkModelNames().back();
    // the id of the object
    attachedObject.object.id = "object cylinder";
    attachedObject.object.primitives.push_back(cylinderPrimitive);
    attachedObject.object.primitive_poses.push_back(grabPose);
    attachedObject.object.operation = attachedObject.object.ADD;
    {
        // lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW planningScene(moveitCppPtr->getPlanningSceneMonitor());
        planningScene->processAttachedCollisionObjectMsg(attachedObject);
        // unlock PlanningScene
    }

    // re-call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    planSolution07 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution07)
    {
        visualTools.publishText(textPose, "obstacle goal with attached object", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartStatePtr->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal07.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution07.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to exit the demo");

    // it is removed from scene indeed, but still visible in rviz
    // it would disappear if another planning follows
    collisionObject.operation = collisionObject.REMOVE;
    {
        // lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW planningScene(moveitCppPtr->getPlanningSceneMonitor());
        planningScene->processCollisionObjectMsg(collisionObject);
        // unlock PlanningScene
    }
    attachedObject.object.REMOVE;
    {
        // lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW planningScene(moveitCppPtr->getPlanningSceneMonitor());
        planningScene->processAttachedCollisionObjectMsg(attachedObject);
        // unlock PlanningScene
    }

    ros::shutdown();
    return 0;
}
