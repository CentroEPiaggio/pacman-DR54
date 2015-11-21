// system
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>

// moveit
// Important: these need to be included before the decision_making includes
//            due to bad use of "using namespace" within decision_making
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/move_group_interface/move_group.h>

// experimental Talking robot
#include <sound_play/sound_play.h>

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

// not the best, but decision_making forces to use global variables
// these are types for data exchange

// from object model to exploration
geometry_msgs::Point sampledPoint;
geometry_msgs::Vector3 sampledNOrmal;

// from exploration to model update
geometry_msgs::Point touchedPoint;

// pulbisher to talk to the user
ros::Publisher talker;

// open/close hand commands (HARDCODE ALERT)
trajectory_msgs::JointTrajectory openHand;
trajectory_msgs::JointTrajectory closeHand;
ros::Publisher hand_commander;

//////////////////////////////////////////////////////
//////////////////      TASKs       //////////////////
//////////////////////////////////////////////////////
decision_making::TaskResult homeTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Homing all devices...");
    ros::Duration polite_timer(1.0);

    // open hand
    hand_commander.publish(openHand);

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
    polite_timer.sleep();
    ros::service::call( std::string("/start_glove_calibration"), empty_srv);
    polite_timer.sleep();

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
    polite_timer.sleep();
    ros::service::call( std::string("/next_orientation"), empty_srv);
    polite_timer.sleep();

    // come back to the 1st move for glove calib
    left_group.setNamedTarget("glove_calib_1");

    // call the 1st move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 1st move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    polite_timer.sleep();
    ros::service::call( std::string("/set_world_reference"), empty_srv);
    polite_timer.sleep();

    // and go home again
    left_group.setNamedTarget("left_arm_home");

    // call the 1st move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 1st move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    ROS_INFO("HEY; I'm at HOME !! You can go by publishing /Start");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult grabTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Please, would you hand me an object?");
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");

    // configure the grab move
    left_group.setNamedTarget("left_arm_pick");

    // call the 1st move for glove calib
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during grabbing movement. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // Use sound to guide the user know
    sound_play::SoundRequest polite_question;
    polite_question.arg = "Please, would you hand me an object?";
    polite_question.command = sound_play::SoundRequest::PLAY_ONCE;
    polite_question.sound = sound_play::SoundRequest::SAY;
    talker.publish(polite_question);

    // Unfortunately, this task is blocking since it's the only one with human interaction
    cout << "Press ENTER to continue after an object has been added..." << endl;
    cin.get();

    hand_commander.publish(closeHand);
    ros::Duration(1.2).sleep();

    sound_play::SoundRequest polite_answer;
    polite_answer.arg = "Thanks!";
    polite_answer.command = sound_play::SoundRequest::PLAY_ONCE;
    polite_answer.sound = sound_play::SoundRequest::SAY;
    talker.publish(polite_answer);

    eventQueue.riseEvent("/ObjectGrabbed");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult handTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Recognizing the hand posture...");
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");
    // timer to allow the filter convergence
    // ToDo: can this be automated? check the diff in joint angles for instance?
    ros::Duration convergence_timer(3.0);

    // configure and call three or four moves to allow filter convergence in static

    left_group.setNamedTarget("left_arm_home");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during grabbing movement. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    left_group.setNamedTarget("glove_calib_1");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during grabbing movement. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    left_group.setNamedTarget("glove_calib_2");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during grabbing movement. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    // the last one is that ready to use vision
    left_group.setNamedTarget("left_arm_peek");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during grabbing movement. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    eventQueue.riseEvent("/HandRecognized");

    return TaskResult::SUCCESS();
}
/*
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

decision_making::TaskResult emergencyStop(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    return TaskResult::SUCCESS();
}*/

//////////////////////////////////////////////////////
//////////////////      SubFSMs       ////////////////
//////////////////////////////////////////////////////

FSM(Start)
{
    FSM_STATES
    {
        GrabObjectFromUser,
        RecognizeHandPosture

    }
    FSM_START(GrabObjectFromUser);
    FSM_BGN
    {
        FSM_STATE(GrabObjectFromUser)
        {
            FSM_CALL_TASK(grabObject);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/ObjectGrabbed", FSM_NEXT(RecognizeHandPosture));
            }
        }
        FSM_STATE(RecognizeHandPosture)
        {
            FSM_CALL_TASK(recognizeHand);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/HandRecognized", FSM_RAISE("/ReadyToModel"));
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
        Start,
        ObjectModelling,
        TactileExploration
    }
    FSM_START(Off);
    FSM_BGN
    {
        FSM_STATE(Off)
        {
            // FSM_CALL_TASK(emergencyStop);

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

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/Start", FSM_NEXT(Start));
            }
        }
        FSM_STATE(Start)
        {
            FSM_CALL_FSM(Start)
            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ReadyToModel", FSM_NEXT(ObjectModelling));
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

    // sound client
    talker = nodeHandle.advertise<sound_play::SoundRequest>("/robotsound", 5);

    // useful hand and close commands in 1sec and publisher
    // ToDo: move it to moveit! to use the move command
    openHand.joint_names.push_back("left_hand_synergy_joint");
    closeHand.joint_names.push_back("left_hand_synergy_joint");
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(0);
    p.time_from_start = ros::Duration(1.0);
    openHand.points.push_back(p);
    p.positions.at(0) = 1;
    closeHand.points.push_back(p);
    hand_commander = nodeHandle.advertise<trajectory_msgs::JointTrajectory>("/left_hand/joint_trajectory_controller/command",1);

    // 2: Register tasks
    LocalTasks::registrate("Home", homeTask);
    LocalTasks::registrate("grabObject", grabTask);
    LocalTasks::registrate("recognizeHand", handTask);

    // 3: Go!
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("Starting DR54 demo...");
    eventQueue.async_spin();
    FsmDR54Logic(NULL, &eventQueue);

    spinner.stop();

    return 0;
}


