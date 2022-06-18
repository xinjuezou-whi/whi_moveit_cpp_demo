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
    std::cout << "\nWHI MoveIt MoveItCpp demo VERSION 00.04" << std::endl;
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
    double paramIkTimeout02 = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan02/ik_timeout", paramIkTimeout02, 0.5);
    std::vector<double> paramStartPose02;
    nodeHandle.getParam("/moveit_cpp_demo/plan02/start_pose", paramStartPose02);
    // plan03
    std::vector<double> paramGoal03;
    nodeHandle.getParam("/moveit_cpp_demo/plan03/goal_pose", paramGoal03);
    // plan04
    std::string paramStateGroup;
    nodeHandle.param("/moveit_cpp_demo/plan04/state_group", paramStateGroup, std::string("ready"));
    // plan05
    double paramIkTimeout05 = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan05/ik_timeout", paramIkTimeout05, 0.5);
    std::vector<double> paramStartPose05;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/start_pose", paramStartPose05);
    std::vector<std::string> paramPoseIndex01;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/pose_index_01", paramPoseIndex01);
    std::vector<double> paramPoseDelta01;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/pose_delta_01", paramPoseDelta01);
    std::vector<std::string> paramPoseIndex02;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/pose_index_02", paramPoseIndex02);
    std::vector<double> paramPoseDelta02;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/pose_delta_02", paramPoseDelta02);
    std::vector<std::string> paramPoseIndex03;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/pose_index_03", paramPoseIndex03);
    std::vector<double> paramPoseDelta03;
    nodeHandle.getParam("/moveit_cpp_demo/plan05/pose_delta_03", paramPoseDelta03);
    double paramJumpThreshold = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan05/jump_threshold", paramJumpThreshold, 0.0);
    double paramEndEffectorStep = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan05/end_effector_step", paramEndEffectorStep, 0.01);
    // plan06
    std::vector<double> paramGoal06;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/goal_pose", paramGoal06);
    std::vector<double> paramBoxSize;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/block_box_size", paramBoxSize);
    std::vector<double> paramBoxPose;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/block_box_pose", paramBoxPose);
    double paramCylinderRadius = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan06/grab_cylinder_radius", paramCylinderRadius, 0.02);
    double paramCylinderHeight = 0.0;
    nodeHandle.param("/moveit_cpp_demo/plan06/grab_cylinder_height", paramCylinderHeight, 0.1);
    std::vector<std::string> paramCylinderPoseIndex;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/grap_cylinder_pose_index", paramCylinderPoseIndex);
    std::vector<double> paramCylinderPose;
    nodeHandle.getParam("/moveit_cpp_demo/plan06/grap_cylinder_pose", paramCylinderPose);

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

    geometry_msgs::Pose robotStartPose;//= tf2::toMsg(robotStartState->getGlobalLinkTransform(paramVisualLink)); // failed to compile why?
    tf::poseEigenToMsg(robotStartState->getGlobalLinkTransform(paramVisualLink), robotStartPose);
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
        visualTools.publishAxisLabeled(robotStartState->getGlobalLinkTransform(paramVisualLink), "start_pose");
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

    /// plan 02
    //
    // the current state of the plan can be set by using moveit::core::RobotState
    auto startState02 = *(moveitCppPtr->getCurrentState());
    geometry_msgs::Pose startPose02;
    startPose02.orientation.w = 1.0;
    if (dof > 6)
    {
        // DOF 7
        startPose02.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        startPose02.orientation = robotStartPose.orientation;
    }
    startPose02.position.x = paramStartPose02[0];
    startPose02.position.y = paramStartPose02[1];
    startPose02.position.z = paramStartPose02[2];

    if (startState02.setFromIK(jointModelGroupPtr, startPose02, paramIkTimeout02))
    {
        planningComponents->setStartState(startState02);

        // reuse the previous goal that we had and plan to it
        auto planSolution02 = planningComponents->plan();
        if (planSolution02)
        {
            moveit::core::RobotState robotState(robotModelPtr);
            moveit::core::robotStateMsgToRobotState(planSolution02.start_state, robotState);

            visualTools.publishText(textPose, "single goal with pre-set start state", rvt::WHITE, rvt::XLARGE);
            visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform(paramVisualLink), "start_pose");
            visualTools.publishAxisLabeled(goal01.pose, "target_pose");
            visualTools.publishTrajectoryLine(planSolution02.trajectory, jointModelGroupPtr);
            visualTools.trigger();

            // uncomment if you want to execute the plan
            //planningComponents->execute();
        }
    }
    else
    {
        visualTools.publishText(textPose, "failed to get the IK from pre-set start state", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(goal01.pose, "target_pose");
        visualTools.trigger();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 03");

    /// plan 03
    //
    // the goal of the plan can be set by using moveit::core::RobotState
    auto targetState = *robotStartState;
    geometry_msgs::Pose goal03;
    if (dof > 6)
    {
        // DOF 7
        goal03.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal03.orientation = robotStartPose.orientation;
    }
    goal03.position.x = paramGoal03[0];
    goal03.position.y = paramGoal03[1];
    goal03.position.z = paramGoal03[2];

    targetState.setFromIK(jointModelGroupPtr, goal03);

    planningComponents->setGoal(targetState);

    // reuse the previous start state that we had and plan from it
    auto planSolution03 = planningComponents->plan();
    if (planSolution03)
    {
        moveit::core::RobotState robotState(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(planSolution03.start_state, robotState);

        visualTools.publishText(textPose, "single goal set by target state", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform(paramVisualLink), "start_pose");
        visualTools.publishAxisLabeled(goal03, "target_pose");
        visualTools.publishTrajectoryLine(planSolution03.trajectory, jointModelGroupPtr);
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
    planningComponents->setGoal(paramStateGroup);

    // reuse the previous start state that we had and plan from it
    auto planSolution04 = planningComponents->plan();
    if (planSolution04)
    {
        moveit::core::RobotState robotState(robotModelPtr);
        moveit::core::robotStateMsgToRobotState(planSolution04.start_state, robotState);

        visualTools.publishText(textPose, "goal from state group", rvt::WHITE, rvt::XLARGE);
        visualTools.publishAxisLabeled(robotState.getGlobalLinkTransform(paramVisualLink), "start_pose");
        moveit::core::RobotState robotTrajLastState = planSolution04.trajectory->getLastWayPoint();
        visualTools.publishAxisLabeled(robotTrajLastState.getGlobalLinkTransform(paramVisualLink), "target_pose");
        visualTools.publishTrajectoryLine(planSolution04.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 05");

    // plan05
    // 
    // restore the start state
    planningComponents->setStartState(*robotStartState);
    // set the state to the one which satisfies the constraints of Cartesian
    auto startState05 = *(moveitCppPtr->getCurrentState());
    geometry_msgs::Pose startPose05;
    if (dof > 6)
    {
        // DOF 7
        startPose05.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        startPose05.orientation = robotStartPose.orientation;
    }
    startPose05.position.x = paramStartPose05[0];
    startPose05.position.y = paramStartPose05[1];
    startPose05.position.z = paramStartPose05[2];
    if (startState05.setFromIK(jointModelGroupPtr, startPose05, paramIkTimeout05))
    {
        planningComponents->setStartState(startState05);

        // organize waypoints
        EigenSTL::vector_Isometry3d waypoints;
        Eigen::Isometry3d eigenPose;
        tf::poseMsgToEigen(startPose05, eigenPose);
        waypoints.push_back(eigenPose);

        geometry_msgs::Pose wayPose = startPose05;
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
        double fraction = startState05.computeCartesianPath(jointModelGroupPtr, resTraj, jointModelGroupPtr->getLinkModel(paramVisualLink), waypoints,
            true, paramEndEffectorStep, paramJumpThreshold);
        ROS_INFO_NAMED("demo", "visualizing plan 05 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
        
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
        visualTools.publishAxisLabeled(startPose05, "target_pose");
        visualTools.trigger();
    }
    
    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with plan 06");

    /// plan 06
    //
    // restore the start state
    planningComponents->setStartState(*robotStartState);
    // first plan a clear goal
    geometry_msgs::PoseStamped goal06;
    goal06.header.frame_id = paramGoalFrame;
    if (dof > 6)
    {
        // DOF 7
        goal06.pose.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal06.pose.orientation = robotStartPose.orientation;
    }
    goal06.pose.position.x = paramGoal06[0];
    goal06.pose.position.y = paramGoal06[1];
    goal06.pose.position.z = paramGoal06[2];
    planningComponents->setGoal(goal06, paramGoalLink);

    // call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    auto planSolution06 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution06)
    {
        visualTools.publishText(textPose, "clear goal", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartState->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal06.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution06.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with next step of plan 06");

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
        // lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW planningScene(moveitCppPtr->getPlanningSceneMonitor());
        planningScene->processCollisionObjectMsg(collisionObject);
        // unlock PlanningScene
    }

    // re-call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    planSolution06 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution06)
    {
        visualTools.publishText(textPose, "obstacle goal", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartState->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal06.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution06.trajectory, jointModelGroupPtr);
        visualTools.trigger();

        // uncomment if you want to execute the plan
        //planningComponents->execute();
    }

    // start the next plan
    visualTools.deleteAllMarkers();
    visualTools.prompt("Press 'next' to continue with next step of plan 06");

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
    moveit_msgs::CollisionObject object2Attach;
    // frame_id decides the coord frame the object belongs to
    object2Attach.header.frame_id = jointModelGroupPtr->getLinkModelNames().back();
    // the id of the object is used to identify it
    object2Attach.id = "object cylinder";
    object2Attach.primitives.push_back(cylinderPrimitive);
    object2Attach.primitive_poses.push_back(grabPose);
    object2Attach.operation = object2Attach.ADD;
    moveit_msgs::AttachedCollisionObject attachedObject;
    attachedObject.object = object2Attach;
    attachedObject.link_name = jointModelGroupPtr->getLinkModelNames().back();
    {
        // lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW planningScene(moveitCppPtr->getPlanningSceneMonitor());
        planningScene->processAttachedCollisionObjectMsg(attachedObject);
        // unlock PlanningScene
    }

    // re-call the PlanningComponents to compute the plan and visualize it
    // note that it is just planning
    planSolution06 = planningComponents->plan();
    // check if PlanningComponents succeeded in finding the plan
    if (planSolution06)
    {
        visualTools.publishText(textPose, "obstacle goal with attached object", rvt::WHITE, rvt::XLARGE);
        // visualize the start pose in rviz
        visualTools.publishAxisLabeled(robotStartState->getGlobalLinkTransform(paramVisualLink), "start_pose");
        // visualize the goal pose in rviz
        visualTools.publishAxisLabeled(goal06.pose, "target_pose");
        // visualize the trajectory in rviz
        visualTools.publishTrajectoryLine(planSolution06.trajectory, jointModelGroupPtr);
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
