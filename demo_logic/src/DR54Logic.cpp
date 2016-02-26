// system
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/SwitchController.h>
#include <std_srvs/Empty.h>

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

// object modelling services
#include "gp_regression/StartProcess.h"
#include "gp_regression/GetNextBestPath.h"
#include "gp_regression/Update.h"

// kdl
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainiksolverpos_lma.hpp>

// contact observer logger
#include "demo_logic/ContactState.h"
#include "demo_logic/GetPathLog.h"

using namespace std;
using namespace decision_making;

// not the best, but decision_making forces to use global variables
// these are types for data exchange.
// IMPORTANT: points must become trajectories or curves in the future

// from object model to exploration
// generate equaly distributed 16 poses with z-axis aligned to normal, and differnt x- and y-axes
KDL::Vector o_ref;
KDL::Vector z_ref;
std::vector<geometry_msgs::PoseStamped> normal_aligned_targets;
int n_div = 16;

// from exploration to model update
gp_regression::Path explored_path;

// pulbisher to talk to the user
ros::Publisher talker;

// open/close hand commands (HARDCODE ALERT)
trajectory_msgs::JointTrajectory openHand;
trajectory_msgs::JointTrajectory closeHand;
ros::Publisher hand_commander;

// possible controller switches, predefined, only for the right arm, the probe
std::string lwr_right_switcher_srv_name;
controller_manager_msgs::SwitchController lwr_right_switcher_srv;
// normal ones
controller_manager_msgs::SwitchControllerRequest fromPosToCart;
controller_manager_msgs::SwitchControllerRequest fromCartToPos;
// safety ones
controller_manager_msgs::SwitchControllerRequest fromPosToGrav;
controller_manager_msgs::SwitchControllerRequest fromGravToPos;

// empty service for anyone
std_srvs::Empty empty_srv;


// only needed for visual debug
ros::Publisher pose_array_pub;

//////////////////////////////////////////////////////
//////////////////      TASKs       //////////////////
//////////////////////////////////////////////////////
decision_making::TaskResult homeTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Homing all devices...");
    ros::Duration polite_timer(1.0);

    // open hand
    hand_commander.publish(openHand);
    polite_timer.sleep();

    // configure the groups
    moveit::planning_interface::MoveGroup two_group("two_arms");
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");

    // ToDO: stop publishing the esimated model

    // and clear octomap
    ros::service::call("clear_octomap", empty_srv);

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

    explored_path.points.clear();
    explored_path.directions.clear();
    explored_path.isOnSurface.clear();

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
        ROS_ERROR("An error occured during recognize hand movement 1. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    left_group.setNamedTarget("glove_calib_1");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during recognize hand movement 2. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    left_group.setNamedTarget("glove_calib_2");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during recognize hand movement 3. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    // the last one is that ready to use vision
    left_group.setNamedTarget("left_arm_peek");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during recognize hand movement 4. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    eventQueue.riseEvent("/HandRecognized");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult createModelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Create the model from visual input...");
    std::string create_model_srv_name = "/gaussian_process/start_process";
    gp_regression::StartProcess create_model_srv;
    create_model_srv.request.obj_pcd = "";
    // create_model_srv.request.obj_pcd = "/home/pacman/Projects/catkin_ws/src/pacman-DR54/gaussian-object-modelling/resources";

    // call the service
    if( !(ros::service::call( create_model_srv_name, create_model_srv) ))
    {
        ROS_ERROR("An error occured during calling the start gp process service. Turnning the logic OFF...");
        // eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    eventQueue.riseEvent("/ObjectModelled");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult updateModelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Update the model with fresh the path from exploration");

    // declare rosservice to update model with explored trajectory

    std::string update_model_srv_name = "update_process";
    gp_regression::Update update_model_srv;
    update_model_srv.request.explored_points = explored_path;
    if( !(ros::service::call(update_model_srv_name, update_model_srv)) )
    {
            ROS_WARN("Could not update the model with fresh data, this does not stop the demo, but get next best action is computed from previous model");
            // eventQueue.riseEvent("/EStop");
            //return TaskResult::TERMINATED();
    }
    eventQueue.riseEvent("/ModelUpdated");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult generateTrajectoryTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Generate a trajectory for exploration according to a pre-selected policy...");
    normal_aligned_targets.clear();
    std::string get_next_best_path_name = "/gaussian_process/get_next_best_path";
    gp_regression::GetNextBestPath get_next_best_path_srv;

    // call the service
    if( !(ros::service::call( get_next_best_path_name, get_next_best_path_srv) ))
    {
        ROS_ERROR("An error occured during calling the sample gp process service. Turnning the logic OFF...");
        // eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // ToDO: take the result and reorient the hand with the object such that the
    // most of the path is well-posed for the probe
    // provide several poses that satisfy this constraint to ease the motion planning

    // convert to the exchange data type
    // TEMP: we now we are dealing with one single point so far
    // ToDO: validate what returns from the gp node
    geometry_msgs::PointStamped startPoint;
    geometry_msgs::Vector3Stamped startNormal;
    startPoint.header = get_next_best_path_srv.response.next_best_path.header;
    startPoint.point = get_next_best_path_srv.response.next_best_path.points.at(0).point;
    startNormal.header = get_next_best_path_srv.response.next_best_path.header;
    startNormal.vector = get_next_best_path_srv.response.next_best_path.directions.at(0).vector;
    // this value should be provided by the gaussian process, the safety distance is inversely proportional
    // to the certainty of the point.
    // ATTENTION: here it is assumed th\at normal is the outward normal
    double safety_distance = 0.05; // related to the octomap offset to allow collision

    // The whole exploration strategy should be provided by the smart policy
    // For the moment, we do the following:
    // Create the reference frame
    o_ref = KDL::Vector(startPoint.point.x, startPoint.point.y, startPoint.point.z);
    z_ref = KDL::Vector(startNormal.vector.x, startNormal.vector.y, startNormal.vector.z);
    z_ref.Normalize();
    o_ref = o_ref + safety_distance*z_ref;
    z_ref = -1*z_ref;
    KDL::Vector y_ref(0.0, 1.0, 0.0);
    KDL::Vector x_ref = y_ref*z_ref;
    x_ref.Normalize();
    y_ref = z_ref*x_ref;
    y_ref.Normalize();
    KDL::Rotation q_ref(x_ref, y_ref, z_ref);
    KDL::Frame g_ref(q_ref, o_ref);

    // only needed for visual debug
    geometry_msgs::PoseArray normal_aligned_array;
    normal_aligned_array.header = startPoint.header;

    for(int i = 0; i < n_div; ++i)
    {
        double current_angle = ((double)i/(double)n_div)*6.283185307179586; //2*pi
        KDL::Rotation current_orientation;
        current_orientation.DoRotZ(current_angle);
        KDL::Frame current_frame (current_orientation);
        current_frame = g_ref*current_frame;

        // convert to geometry msg
        geometry_msgs::PoseStamped current_target;
        tf::poseKDLToMsg(current_frame, current_target.pose);
        current_target.header = startPoint.header;
        normal_aligned_targets.push_back(current_target);

        // only needed for visual debug
        normal_aligned_array.poses.push_back(current_target.pose);
    }

    // only needed for visual debug
    pose_array_pub.publish(normal_aligned_array);

    eventQueue.riseEvent("/PolicyGenerated");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult moveCloserTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Approaches to the surface according to exploration policy...");

    // the dismembered, a single (two arm) chain
    KDL::Chain dismembered;

    // configure the left arm hand group
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");

    // first, get the urdf from the group
    // note that, it could be any group, this is common for everyone
    robot_model::RobotModelConstPtr vito_model = left_group.getRobotModel();
    boost::shared_ptr<const urdf::ModelInterface> vito_urdf = vito_model->getURDF();
    KDL::Tree vito_kdl;
    kdl_parser::treeFromUrdfModel( *vito_urdf, vito_kdl );

    // get the left arm chain (normal), from common root to tip
    KDL::Chain left_group_chain_kdl;
    vito_kdl.getChain( vito_model->getRootLinkName(), left_group.getEndEffectorLink(), left_group_chain_kdl );

    KDL::JntArray left_group_Qi(left_group_chain_kdl.getNrOfJoints());
    left_group_Qi.data = Eigen::Map<Eigen::VectorXd>( (double *)left_group.getCurrentJointValues().data(),
                                                      left_group.getCurrentJointValues().size());

    // ToDO: add joint limits

    // start building the dismemebered chain
    dismembered.addChain(left_group_chain_kdl);

    // set the virtual joint in between using the computed point and axis in safety distance from the surface
    // the o_ref and z_ref comes from the modelling
    o_ref = KDL::Vector(0,0,0.3);
    z_ref = KDL::Vector(0,0,1);
    KDL::Joint touch_joint("touch_joint",
                           o_ref,
                           z_ref,
                           KDL::Joint::RotAxis);
    KDL::Frame touch_tip = touch_joint.pose(0);
    // rotate the tip so that z points inward
    touch_tip.M.DoRotY(3.141592);
    KDL::Segment touch_segment( "touch_segment", touch_joint, touch_tip);
    KDL::JntArray touch_Qi(1);
    touch_Qi(0) = 0;

    // keep building the dismembered chain
    dismembered.addSegment( touch_segment );

    // get the touch chain from the group (in reverse)
    moveit::planning_interface::MoveGroup touch_group("touch_chain");
    touch_group.setEndEffector("touch");

    // now get the dismembered arm, from tip to the common root
    // note here we go from the end effector to the root, so the goal will be the identity
    KDL::Chain touch_group_chain_kdl;
    vito_kdl.getChain( vito_model->getRootLinkName(), touch_group.getEndEffectorLink(), touch_group_chain_kdl );

    KDL::Chain touch_group_chain_kdl_reverse;
    vito_kdl.getChain( touch_group.getEndEffectorLink(), vito_model->getRootLinkName(), touch_group_chain_kdl_reverse );

    // reverse the orders of the initial guess
    std::vector<double> current_touch_group_values = touch_group.getCurrentJointValues();
    std::reverse(current_touch_group_values.begin(), current_touch_group_values.end());
    KDL::JntArray touch_group_Qi(touch_group_chain_kdl_reverse.getNrOfJoints());
    touch_group_Qi.data = Eigen::Map<Eigen::VectorXd>( (double *)current_touch_group_values.data(),
                                                      current_touch_group_values.size());

    // ToDO: add joint limits

    // and keep building the dismembered two-arm chain
    dismembered.addChain(touch_group_chain_kdl_reverse);

    // solve IK of the dismembered with goal being the identity to close the loop
    // 1 attempt prototype

    // Our target is creating a single (two arm chain) to be closed, so our goal is the identity
    KDL::Frame target = KDL::Frame::Identity();

    KDL::JntArray Qf( dismembered.getNrOfJoints() );

    // fill initial guess with current state
    KDL::JntArray Qi( dismembered.getNrOfJoints() );

    // going to eigen level to concatenate
    Qi.data.block(0,0, left_group_Qi.rows(), 1) = left_group_Qi.data;
    Qi.data.block(left_group_Qi.rows(), 0, touch_Qi.rows(), 1) = touch_Qi.data;
    Qi.data.block(left_group_Qi.rows() + touch_Qi.rows(), 0, touch_group_Qi.rows(), 1) = touch_group_Qi.data;

    // ToDO: add limits
    KDL::ChainIkSolverPos_LMA ik_solver(dismembered);
    int res = ik_solver.CartToJnt(Qi, target, Qf);

    // ToDO: add several attempts randomizing Qi
    if (res == 0)
            cout << "YEY !" << endl;
    else
            cout << "BOO :( -> several attempts is WIP" << endl;

    // configure the joint target as a map to avoid joint missordering
    std::map<std::string, double> solution;
    int j = 0;
    for( auto seg: dismembered.segments )
    {
        // recall to avoid the virtual joint and do the +1 joint for the reversed chain
        if( seg.getJoint().getTypeName().compare("None") != 0 )
        {
            if( seg.getJoint().getName().compare("touch_joint") !=0 )
            {
                // ToDO: avoid this magic number
                if( j < 7 )
                    solution.insert( std::make_pair( seg.getJoint().getName(), Qf(j) ) );
                else
                     solution.insert( std::make_pair( seg.getJoint().getName(), Qf(j+1) ) );
                j++;
            }
            else if( seg.getJoint().getName().compare("touch_joint") == 0 )
            {
                cout << seg.getJoint().getName() << " = " << Qf(j+1) << endl;
            }
        }
    }

    // configure dual group and move, set solution target, and go!
    moveit::planning_interface::MoveGroup two_group("two_arms");
    two_group.setJointValueTarget(solution);
    two_group.setMaxVelocityScalingFactor(0.5);
    if( !(two_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured approaching to surface. Coming back to object modelling...");
        eventQueue.riseEvent("/DidNotExplore");
        return TaskResult::TERMINATED();
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult touchObjectTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Touching the object...");
    // ros::Duration switch_timer(5.0);

    // swtich to cartesian impedance control
    /*lwr_right_switcher_srv.request = fromPosToCart;
    if( !(ros::service::call( lwr_right_switcher_srv_name, lwr_right_switcher_srv)) )
    {
            ROS_ERROR("I could not switch to Cartesian impedance controller...");
            eventQueue.riseEvent("/EStop");
            return TaskResult::TERMINATED();
    }
    switch_timer.sleep();

    ROS_INFO("TESTING CARTESIAN IMPEDANCE CONTROLLER");
    cin.get();*/

    // set cartesian impedance parameters

    // and read from next best path and go even like very vast trajectory with sleeps!

    // FOR NOW, MOVE AGAIN FORWARD TO CONTACT WITH MOVEIT
    double safety_distance = 0.05;

    // configure, set targets and work frames, and move the group
    moveit::planning_interface::MoveGroup touch_group("touch_chain");
    touch_group.setEndEffector("touch");
    geometry_msgs::PoseStamped touch_target = touch_group.getCurrentPose(touch_group.getEndEffectorLink());
    KDL::Frame touch_pose;
    tf::poseMsgToKDL(touch_target.pose, touch_pose);
    touch_pose.p = touch_pose.p + safety_distance*touch_pose.M.UnitZ();
    tf::poseKDLToMsg(touch_pose, touch_target.pose);

    touch_group.setPoseTarget(touch_target, touch_group.getEndEffectorLink());

    if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured moving towards surface. Coming back to object modelling...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    eventQueue.riseEvent("/DoneExploration");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult getPathLogTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
        ROS_INFO("Getting the explored path. Filtering and checking might be done here");
        std::string path_log_srv_name = "/get_path_log";

        demo_logic::GetPathLog get_path_log_srv;

        if( !( ros::service::call( path_log_srv_name, get_path_log_srv )) )
        {
                ROS_WARN("Couldn't get the log of the explored path, rising didn't explore event");
                eventQueue.riseEvent("/DidNotExplore");
                return TaskResult::TERMINATED();
        }

        // filter, check zeroes, discard close points, etc, etc...
        explored_path = get_path_log_srv.response.path_log;

        return TaskResult::SUCCESS();
}


decision_making::TaskResult moveAwayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Moves away from the surface...");
    ros::Duration switch_timer(5.0);

    // swtich to joint position control
    // lwr_right_switcher_srv.request = fromCartToPos;
    /*lwr_right_switcher_srv.request = fromGravToPos; // TEST WITH GRAV
    if( !ros::service::call( lwr_right_switcher_srv_name, lwr_right_switcher_srv) )
    {
            ROS_ERROR("I could not switch to Position controller...");
            eventQueue.riseEvent("/EStop");
            return TaskResult::TERMINATED();
    }
    switch_timer.sleep();*/

    // this value should be provided by the gaussian process, the safety distance is inversely proportional
    // to the certainty of the point.
    // ATTENTION: here it is assumed that z always points toward the surface
    double safety_distance = 0.05;

    // configure, set targets and work frames, and move the group
    moveit::planning_interface::MoveGroup touch_group("touch_chain");
    touch_group.setEndEffector("touch");
    geometry_msgs::PoseStamped retreat_target = touch_group.getCurrentPose(touch_group.getEndEffectorLink());
    KDL::Frame retreat_pose;
    tf::poseMsgToKDL(retreat_target.pose, retreat_pose);
    retreat_pose.p = retreat_pose.p - safety_distance*retreat_pose.M.UnitZ();
    tf::poseKDLToMsg(retreat_pose, retreat_target.pose);

    touch_group.setPoseTarget(retreat_target, touch_group.getEndEffectorLink());

    if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured moving away from surface. Coming back to object modelling...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    ROS_INFO("Now, take probe to home...");

    // configure the group
    moveit::planning_interface::MoveGroup right_group("right_arm_probe");

    // configure the home move
    right_group.setNamedTarget("right_arm_home");

    // call the move
    if( !(right_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured moving the probe to home. But exploration was succesful, retry moving away again...");
        eventQueue.riseEvent("/DoneExploration");
        return TaskResult::TERMINATED();
    }

    eventQueue.riseEvent("/FinishedExploration");

    return TaskResult::SUCCESS();
}


decision_making::TaskResult emergencyStopTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_WARN("EStop called, do something");
    return TaskResult::SUCCESS();
}

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
                // Manual queries
                FSM_ON_EVENT("/GenerateModel", FSM_NEXT(GaussianModel));
                FSM_ON_EVENT("/GeneratePolicy", FSM_NEXT(ExplorationStrategy));
                FSM_ON_EVENT("/UpdateModel", FSM_NEXT(UpdateModel));
                FSM_ON_EVENT("/QueryDone", FSM_RAISE("/ObjectModellingDone"));

                // Auto queries
                // In case it all went well with the exploration
                FSM_ON_EVENT("/FinishedExploration", FSM_NEXT(UpdateModel));

                // In case something went wrong with the exploration, change strategy
                FSM_ON_EVENT("/DidNotExplore", FSM_NEXT(ExplorationStrategy));

                // Once the exploration measure gives that the object is complete
                // return the object, for now, go home.
                FSM_ON_EVENT("/ModelCompleted", FSM_RAISE("/ReturnObject"));
            }
        }
        FSM_STATE(GaussianModel)
        {
            FSM_CALL_TASK(createModel);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/ObjectModelled", FSM_NEXT(ExplorationStrategy));
            }
        }
        FSM_STATE(UpdateModel)
        {
            FSM_CALL_TASK(updateModel);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/ModelUpdated", FSM_NEXT(ExplorationStrategy));
            }
        }
        FSM_STATE(ExplorationStrategy)
        {
            FSM_CALL_TASK(toExploreTrajectory);

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
        ApproachToSurface,
        TactileExploration,
        RetreatFromSurface
    }
    FSM_START(ApproachToSurface);
    FSM_BGN
    {
        FSM_STATE(ApproachToSurface)
        {
            // Move with position control and planning, in free space, close to the surface with:
            //    * The z-axis of tip parallel to surface normal at desired contact point
            //    * Displace according to model uncertainty along z-axis ?

            FSM_CALL_TASK(moveCloser);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CloseToSurface", FSM_NEXT(TactileExploration));
            }
        }
        FSM_STATE(TactileExploration)
        {
            // Move using force impedance control using:
            //    * ~5N in z-axis of tip (depends on the sensor capabilities)
            //    * With low-stiffness in z-axis, the rest are stiff
            FSM_CALL_TASK(touchIt);

            ROS_INFO("Engage touching controller and send command, and raise event when touching");

            // Move in Cartesian impedance control with:
            //    * ~5N in z-axis of tip
            //    * With orientation correction to keep z-axis of tip parallel to surface normal
            //    * With low-stiffness in z-axis, the rest are stiff
            //    * xy coordinates follow the trajectory over the surface

            ROS_INFO("Engage contour following controller and send command, and raise event after completion");

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DoneExploration", FSM_NEXT(RetreatFromSurface));
            }
        }
        FSM_STATE(RetreatFromSurface)
        {
            // with position control and planning, away from the surface
            //    * Move away in the z-axis (normal direction) until no contact (~0N along z-axis)
            //    * With high stiffness in all direction return to home position

            FSM_CALL_TASK(moveAway);
            FSM_CALL_TASK(getLog);

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
        TactileExploration,
        End
    }
    FSM_START(Off);
    FSM_BGN
    {
        FSM_STATE(Off)
        {
            FSM_CALL_TASK(emergencyStop);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GoHome", FSM_NEXT(Home));
            }
        }
        FSM_STATE(Home)
        {
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
                FSM_ON_EVENT("/Explore", FSM_NEXT(TactileExploration));
                FSM_ON_EVENT("/ReturnObject", FSM_NEXT(End));
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
        FSM_STATE(End)
        {
            // FSM_CALL_TASK(returnObject)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ObjectReturned", FSM_NEXT(Home));
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

    // only for visual debug
    pose_array_pub = nodeHandle.advertise<geometry_msgs::PoseArray>("/normal_aligned_targets", 16, true);

    // useful hand and close commands in 1sec and publisher
    // ToDo: move it with moveit! to use the move command
    openHand.joint_names.push_back("left_hand_synergy_joint");
    closeHand.joint_names.push_back("left_hand_synergy_joint");
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(0);
    p.time_from_start = ros::Duration(1.0);
    openHand.points.push_back(p);
    p.positions.at(0) = 1;
    closeHand.points.push_back(p);
    hand_commander = nodeHandle.advertise<trajectory_msgs::JointTrajectory>("/left_hand/joint_trajectory_controller/command",1);
    // configure switches
    lwr_right_switcher_srv_name = "/right_arm/controller_manager/switch_controller";
    fromPosToCart.start_controllers.push_back("cartesian_impedance_controller");
    fromPosToCart.stop_controllers.push_back("joint_trajectory_controller");
    fromPosToCart.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;

    fromCartToPos.start_controllers.push_back("joint_trajectory_controller");
    fromCartToPos.stop_controllers.push_back("cartesian_impedance_controller");
    fromCartToPos.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;

    fromPosToGrav.start_controllers.push_back("gravity_compensation_controller");
    fromPosToGrav.stop_controllers.push_back("joint_trajectory_controller");
    fromPosToGrav.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

    fromGravToPos.start_controllers.push_back("joint_trajectory_controller");
    fromGravToPos.stop_controllers.push_back("gravity_compensation_controller");
    fromGravToPos.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;

    // 2: Register tasks
    LocalTasks::registrate("Home", homeTask);
    LocalTasks::registrate("grabObject", grabTask);
    LocalTasks::registrate("recognizeHand", handTask);
    LocalTasks::registrate("createModel", createModelTask);
    LocalTasks::registrate("toExploreTrajectory", generateTrajectoryTask);
    LocalTasks::registrate("updateModel", updateModelTask);
    LocalTasks::registrate("moveCloser", moveCloserTask);
    LocalTasks::registrate("touchIt", touchObjectTask);
    LocalTasks::registrate("moveAway", moveAwayTask);
    LocalTasks::registrate("getLog", getPathLogTask);
    LocalTasks::registrate("emergencyStop", emergencyStopTask);

    // 3: Go!
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("Starting DR54 demo...");
    eventQueue.async_spin();
    FsmDR54Logic(NULL, &eventQueue);

    spinner.stop();

    return 0;
}


