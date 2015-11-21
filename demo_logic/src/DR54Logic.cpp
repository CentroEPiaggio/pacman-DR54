// system
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

// moveit
// Important: these need to be included before the decision_making includes
//            due to bad use of "using namespace" within decision_making
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/move_group_interface/move_group.h>

// decision making package
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

// pacman_vision_comm
#include "pacman_vision_comm/get_cloud_in_hand.h"

// kuka arm

using namespace std;
using namespace decision_making;

//////////////////////////////////////////////////////
//////////////////      TASKs       //////////////////
//////////////////////////////////////////////////////
decision_making::TaskResult segmentTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Calling segmentation service...");
    std:string segmentation_srv_name = "/pacman_vision/listener/get_cloud_in_hand";

    // configure the service
    pacman_vision_comm::get_cloud_in_hand get_cloud_in_hand_srv;
    get_cloud_in_hand_srv.request.right = true;
    get_cloud_in_hand_srv.request.save = "/home/pacman/Projects/catkin_ws/src/pacman-DR54/demo_logic/tmp/";

    // call the service
    if( ros::service::call( segmentation_srv_name, get_cloud_in_hand_srv) )
    {
        eventQueue.riseEvent("/SegmentationServiceCalled");
    }
    else
    {
        ROS_ERROR("An error occured during calling the segmentation service. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult homeTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Homing all devices...");
    ros::Duration calib_timer(5.0);

    // configure the groups
    moveit::planning_interface::MoveGroup two_group("two_arms");
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");
    moveit::planning_interface::MoveGroup right_group("right_arm_probe");

    // configure the home moves
    two_group.setNamedTarget("two_arms_home");

    // call the moves
    if( !(two_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during moving robots to HOME. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // configure and empty service
    std_srvs::Empty empty_srv;

    // configure the 1st move for glove calib
    left_group.setNamedTarget("glove_calib_1");

    // call the 1st move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 1st move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // call glove calib service
    ros::service::call( std::string("/start_glove_calibration"), empty_srv);
    // calib_timer.sleep();

    // configure the 2nd move for glove calib
    left_group.setNamedTarget("glove_calib_2");

    // call the 2nd move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 2nd move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // call glove 2nd calib service
    ros::service::call( std::string("/next_orientation"), empty_srv);
    // calib_timer.sleep();

    // come back to the 1st move for glove calib
    left_group.setNamedTarget("glove_calib_1");

    // call the 1st move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 1st move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    ros::service::call( std::string("/set_world_reference"), empty_srv);
    //calib_timer.sleep();

    // and go home again
    left_group.setNamedTarget("left_arm_home");

    // call the 1st move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 1st move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult createModelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    return TaskResult::SUCCESS();
}

decision_making::TaskResult updateModelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    return TaskResult::SUCCESS();
}

decision_making::TaskResult generateExplorationTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    return TaskResult::SUCCESS();
}

//////////////////////////////////////////////////////
//////////////////      SubFSMs       ////////////////
//////////////////////////////////////////////////////

FSM(InHandObjectSegmentation)
{
    FSM_STATES
    {
        SegmentingObjectInHand,
        SavingToFile

    }
    FSM_START(SegmentingObjectInHand);
    FSM_BGN
    {
        FSM_STATE(SegmentingObjectInHand)
        {

            FSM_CALL_TASK(SegmentInHand);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/SegmentationServiceCalled", FSM_NEXT(SavingToFile));
            }
        }
        FSM_STATE(SavingToFile)
        {
            ROS_WARN("[MATLAB] Write the segmented cloud and check quality before modelling");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GoodSegmentation", FSM_RAISE("/ObjectSegmented"));
                FSM_ON_EVENT("/BadSegmentation", FSM_NEXT(SegmentingObjectInHand));
            }
        }
    }
    FSM_END
}

FSM(ObjectModelling)
{
    FSM_STATES
    {
        Query,
        GaussianModel,
        UpdateModel,
        ExplorationStrategy
    }
    FSM_START(Query);
    FSM_BGN
    {
        FSM_STATE(Query)
        {
            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GenerateModel", FSM_NEXT(GaussianModel));
                FSM_ON_EVENT("/GeneratePolicy", FSM_NEXT(ExplorationStrategy));
                FSM_ON_EVENT("/UpdateModel", FSM_NEXT(UpdateModel));
                FSM_ON_EVENT("/QueryDone", FSM_RAISE("/ObjectModellingDone"));

                // Once the exploration measure gives that the object is complete
                FSM_ON_EVENT("/ModelCompleted", FSM_RAISE("/GoHome"));
            }
        }
        FSM_STATE(GaussianModel)
        {
            ROS_WARN("[MATLAB] Read file from segmentation and generate model, write model in file");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/ObjectModelled", FSM_NEXT(Query));
            }
        }
        FSM_STATE(UpdateModel)
        {
            ROS_WARN("[MATLAB] Read file from exploration and update the model, write updated model in file");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/ModelUpdated", FSM_NEXT(Query));
            }
        }
        FSM_STATE(ExplorationStrategy)
        {
            ROS_WARN("[MATLAB] Use the current model to generate a new exploration trajectory to improve model");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/PolicyGenerated", FSM_NEXT(Query));
            }
        }
    }
    FSM_END
}

FSM(TactileExploration)
{
    FSM_STATES
    {
        CheckPolicy,
        ApproachToSurface,
        TouchSurface,
        FollowPolicy,
        RetreatFromSurface
    }
    FSM_START(CheckPolicy);
    FSM_BGN
    {
        FSM_STATE(CheckPolicy)
        {
            ROS_INFO("Plan kinematically the whole trajectory and check validity");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GoodTrajectory", FSM_NEXT(ApproachToSurface));
                FSM_ON_EVENT("/BadTrajectory", FSM_RAISE("/DidNotExplore"));
            }
        }
        FSM_STATE(ApproachToSurface)
        {
            // Move in Cartesian impedance control in free space close to the surface with:
            //    * The z-axis of tip parallel to surface normal at desired contact point
            //    * Displace according to model uncertainty along z-axis
            //    * With high-stiffness in all axes

            ROS_INFO("Engage free space controller and send command");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CloseToSurface", FSM_NEXT(TouchSurface));
            }
        }
        FSM_STATE(TouchSurface)
        {
            // Move using force impedance control using:
            //    * ~5N in z-axis of tip (depends on the sensor capabilities)
            //    * With low-stiffness in z-axis, the rest are stiff

            ROS_INFO("Engage touching controller and send command, and raise event when touching");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/Touching", FSM_NEXT(FollowPolicy));
            }
        }
        FSM_STATE(FollowPolicy)
        {
            // Move in Cartesian impedance control with:
            //    * ~5N in z-axis of tip
            //    * With orientation correction to keep z-axis of tip parallel to surface normal
            //    * With low-stiffness in z-axis, the rest are stiff
            //    * xy coordinates follow the trajectory over the surface

            ROS_INFO("Engage contour following controller and send command, and raise event after completion");

            ROS_INFO("[MATLAB] After completion, save the exploration data such that the model can be updated");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FinishedRun", FSM_NEXT(RetreatFromSurface));
            }
        }
        FSM_STATE(RetreatFromSurface)
        {
            // Move in Cartesian impedance control with:
            //    * Move away in the z-axis until no contact (~0N along z-axis)
            //    * With high stiffness in all direction return to home position

            ROS_INFO("Engage no-contact controller and send commands");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/AwayFromSurface", FSM_RAISE("/FinishedExploration"));
            }

        }
    }
    FSM_END
}

//////////////////////////////////////////////////////
//////////////////      FSM       ////////////////////
//////////////////////////////////////////////////////

FSM(DR54Logic)
{
    FSM_STATES
    {
        Off,
        Home,
        InHandObjectSegmentation,
        ObjectModelling,
        TactileExploration
    }
    FSM_START(Off);
    FSM_BGN
    {
        FSM_STATE(Off)
        {
            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GoHome", FSM_NEXT(Home));
                // FSM_ON_EVENT("/Continue", FSM_NEXT(CurrentState));
            }
        }
        FSM_STATE(Home)
        {
            // Call home function from moveit
            FSM_CALL_TASK(Home);

            ROS_INFO("HEY; I'm at HOME !! You can go by publishing /Start");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/Start", FSM_NEXT(InHandObjectSegmentation));
            }
        }
        FSM_STATE(InHandObjectSegmentation)
        {
            FSM_CALL_FSM(InHandObjectSegmentation)
            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ObjectSegmented", FSM_NEXT(ObjectModelling));
            }
        }
        FSM_STATE(ObjectModelling)
        {
            FSM_CALL_FSM(ObjectModelling)
            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ObjectModellingDone", FSM_NEXT(TactileExploration));
                FSM_ON_EVENT("/GoHome", FSM_NEXT(Home));
            }
        }
        FSM_STATE(TactileExploration)
        {
            FSM_CALL_FSM(TactileExploration)
            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/FinishedExploration", FSM_NEXT(ObjectModelling));
                FSM_ON_EVENT("/DidNotExplore", FSM_NEXT(ObjectModelling));
            }
        }

    }
    FSM_END
}


//////////////////////////////////////////////////////
//////////////////      MAIN       ///////////////////
//////////////////////////////////////////////////////
int main(int argc, char** argv){

    // 1: Init
    ros::init(argc, argv, "DR54_logic");
    ros_decision_making_init(argc, argv);

    ros::NodeHandle nodeHandle("~");
    RosEventQueue eventQueue;

    // 2: Register tasks
    LocalTasks::registrate("SegmentInHand", segmentTask);
    LocalTasks::registrate("Home", homeTask);

    // 3: Go!
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("Starting DR54 demo...");
    eventQueue.async_spin();
    FsmDR54Logic(NULL, &eventQueue);

    spinner.stop();

    return 0;
}


