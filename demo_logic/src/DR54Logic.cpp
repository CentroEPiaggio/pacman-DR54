// system
#include <iostream>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
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
#include "intrinsic_tactile_toolbox/TactileInfo.h"

using namespace std;
using namespace decision_making;

// NOTICE
// not the best, but decision_making forces to use global variables
// these are types for data exchange.

// algorithm parameters
double global_variance;
double expanded_variance;
int reduction_rate;
// when this value is reached, the exploration has not ended
// this is to exit a "local minimum"
// it just moves to a collision-safe location and continues til exploration is done
int attempts_to_reset = 0;
int max_attempts_to_reset = 5;

// from object model to exploration
// generate equaly distributed 16 poses with z-axis aligned to normal, and differnt x- and y-axes
KDL::Vector o_ref;
KDL::Vector z_ref;
KDL::Frame g_ref;
std::vector<geometry_msgs::PoseStamped> normal_aligned_targets;
int n_div = 16;
geometry_msgs::Pose I;
moveit_msgs::AttachedCollisionObject predicted_collider;
ros::Publisher collider_pub;
geometry_msgs::PointStamped empty_point;
gp_regression::GetNextBestPath get_next_best_path_srv;

// from exploration to model update
gp_regression::Path explored_path;

// pulbisher to talk to the user
ros::Publisher talker;

// possible controller switches, predefined, only for the right arm, the probe
std::string lwr_right_switcher_srv_name;
controller_manager_msgs::SwitchController lwr_right_switcher_srv;
// normal ones
controller_manager_msgs::SwitchControllerRequest fromPosToCart;
controller_manager_msgs::SwitchControllerRequest fromCartToPos;
// safety ones
controller_manager_msgs::SwitchControllerRequest fromPosToGrav;
controller_manager_msgs::SwitchControllerRequest fromGravToPos;

// for anyone
std_srvs::Empty empty_srv;
sound_play::SoundRequest voice;

// only needed for visual debug
ros::Publisher pose_array_pub;

// contact observer
ros::Subscriber contact_sub;
geometry_msgs::PointStamped current_contact;
demo_logic::ContactState contact_state;
std::shared_ptr<tf::TransformListener> tf_listener;

// processing frame
std::string processing_frame_name;

// random engine
// First create an instance of an engine.
std::random_device rnd_device;
// Specify the engine and distribution.
std::mt19937 mersenne_engine(rnd_device());
std::uniform_real_distribution<double> dist(-1, 1);
// bind the dist to the random engine to create a generator
auto gen = std::bind(dist, mersenne_engine);

//////////////////////////////////////////////////////
//////////////      ContactMonitor       /////////////
//////////////////////////////////////////////////////
void registerContactInfo(const intrinsic_tactile_toolbox::TactileInfo::Ptr &msg)
{
    current_contact.header = msg->point.header;

    // it is a semisphere of radius 2cm, so this would be enough
    // if no contact, set the probe tip in the same frame
    if( !msg->isTipContact.data )
    {
        contact_state.status = demo_logic::ContactState::NO_CONTACT;
        current_contact = msg->point;
    }
    else
    {
        contact_state.status = demo_logic::ContactState::IN_CONTACT;
        current_contact = msg->point;
    }
}

//////////////////////////////////////////////////////
//////////////////      TASKs       //////////////////
//////////////////////////////////////////////////////
decision_making::TaskResult homeTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Homing all devices...");
    ros::Duration polite_timer(1.0);

    explored_path.points.clear();
    explored_path.directions.clear();
    explored_path.isOnSurface.clear();
    explored_path.distances.clear();

    // dirty hack to remove object from the attached collision object
    predicted_collider.object.operation = moveit_msgs::CollisionObject::REMOVE;
    collider_pub.publish(predicted_collider);
    predicted_collider.object.operation = moveit_msgs::CollisionObject::ADD;
    shape_msgs::Mesh empty;
    predicted_collider.object.meshes.at(0) = empty;
    collider_pub.publish(predicted_collider);

    // configure the groups
    moveit::planning_interface::MoveGroup two_group("two_arms");
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");
    moveit::planning_interface::MoveGroup left_hand("left_hand");

    // tare the force torque sensor
    ros::service::call("/probe_ft_sensor/tare", empty_srv );
    // clear octomap
    ros::service::call("clear_octomap", empty_srv);

    // open the hand
    left_hand.setNamedTarget("open");
    left_hand.move();
    // avoid move check on the hand until we figure out how to set the goal tolerance higher
    /*if( !(left_hand.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured openning the hand. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }*/

    // configure and call the home moves
    two_group.setNamedTarget("two_arms_home");
    two_group.setMaxVelocityScalingFactor(0.25);
    if( !(two_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during moving robots to HOME. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // configure the 1st move for glove calib
    left_group.setNamedTarget("glove_calib_1");
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
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during 1st move for glove calibration. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    // tare the force torque sensor
    ros::service::call("/probe_ft_sensor/tare", empty_srv );

    ROS_INFO("HEY; I'm at HOME !! You can go by publishing /Start");
    voice.arg = "HEY; I'm HOME !!";
    talker.publish(voice);

    return TaskResult::SUCCESS();
}

decision_making::TaskResult getObjectTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Please, would you hand me an object?");
    moveit::planning_interface::MoveGroup left_group("left_arm_hand");
    moveit::planning_interface::MoveGroup left_hand("left_hand");

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

    // close the hand to 100% of full closing at 80% of the speed
    left_hand.setNamedTarget("100closed");
    left_hand.setMaxVelocityScalingFactor(0.8);
    left_hand.move();
    // avoid move check on the hand until we figure out how to set the goal tolerance higher
    /*if( !(left_hand.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured closing the hand. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }*/

    sound_play::SoundRequest polite_answer;
    polite_answer.arg = "Thanks!";
    polite_answer.command = sound_play::SoundRequest::PLAY_ONCE;
    polite_answer.sound = sound_play::SoundRequest::SAY;
    talker.publish(polite_answer);

    ROS_INFO("Recognizing the hand posture...");
    // voice.arg = "I'm recognizing how my hand is using this fancy glove.";
    // talker.publish(voice);

    // moveit::planning_interface::MoveGroup left_group("left_arm_hand");
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
    voice.arg = "Now, let me take a closer look.";
    talker.publish(voice);

    left_group.setNamedTarget("left_arm_peek");
    if( !(left_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured during recognize hand movement 4. Turnning the logic OFF...");
        eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }
    convergence_timer.sleep();

    eventQueue.riseEvent("/ObjectReceived");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult createModelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Create the model from visual input...");

    std::string create_model_srv_name = "/gaussian_process/start_process";
    gp_regression::StartProcess create_model_srv;

    // call the service
    if( !(ros::service::call( create_model_srv_name, create_model_srv) ))
    {
        ROS_ERROR("An error occured during calling the start gp process service. Turnning the logic OFF...");
        // eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
    }

    predicted_collider.object.meshes.at(0) = create_model_srv.response.predicted_shape;
    predicted_collider.object.operation = moveit_msgs::CollisionObject::ADD;
    collider_pub.publish(predicted_collider);

    eventQueue.riseEvent("/ObjectModeled");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult updateModelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Update the model with the fresh path from exploration");

    std::string update_model_srv_name = "/gaussian_process/update_process";
    gp_regression::Update update_model_srv;
    update_model_srv.request.explored_points = explored_path;
    if( !(ros::service::call(update_model_srv_name, update_model_srv)) )
    {
            ROS_WARN("Could not update the model with fresh data, this does not stop the demo, but get next best action is computed from previous model");
            // eventQueue.riseEvent("/EStop");
            return TaskResult::TERMINATED();
    }

    // clear the explored path
    explored_path.points.clear();
    explored_path.directions.clear();
    explored_path.isOnSurface.clear();
    explored_path.distances.clear();

    // add the estimated shape as collision object
    predicted_collider.object.meshes.at(0) = update_model_srv.response.predicted_shape;
    predicted_collider.object.operation = moveit_msgs::CollisionObject::ADD;
    collider_pub.publish(predicted_collider);

    eventQueue.riseEvent("/ModelUpdated");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult generateTrajectoryTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Generate a trajectory for exploration according to a pre-selected policy...");

    normal_aligned_targets.clear();
    std::string get_next_best_path_name = "/gaussian_process/get_next_best_path";

    // the gp_atlas_rrt loop
    while( true )
    {
        get_next_best_path_srv.request.var_desired.data = static_cast<float>(expanded_variance);

        // call the service
        if( !(ros::service::call( get_next_best_path_name, get_next_best_path_srv) ))
        {
        ROS_ERROR("An error occured during calling the sample gp process service. Turnning the logic OFF...");
        // eventQueue.riseEvent("/EStop");
        return TaskResult::TERMINATED();
        }

        // the ending condition of the algorithm
        if( get_next_best_path_srv.response.next_best_path.points.empty() )
        {
            if( expanded_variance > global_variance )
            {
                expanded_variance = reduction_rate*expanded_variance < global_variance ? global_variance : reduction_rate*expanded_variance;
            }
            else
            {
                ROS_INFO("I'm done with the tactile exploration !...");
                voice.arg = "I'm done with the tactile exploration !...";
                talker.publish(voice);
                eventQueue.riseEvent("/GoodObjectModel");
                return TaskResult::TERMINATED();
            }
        }
        else
        {
            break;
        }
    }

    // // Create the reference frame for touching
    geometry_msgs::PointStamped startPoint;
    geometry_msgs::Vector3Stamped startNormal;
    startPoint.header = get_next_best_path_srv.response.next_best_path.header;
    startPoint.point = get_next_best_path_srv.response.next_best_path.points.back().point;
    startNormal.header = get_next_best_path_srv.response.next_best_path.header;
    startNormal.vector = get_next_best_path_srv.response.next_best_path.directions.back().vector;
    // this value could be provided by the gaussian process, the safety distance could be inversely proportional
    // to the certainty of the point
    // ATTENTION: here it is assumed that normal is in the outward direction
    double safety_distance = 0.05; // related to the octomap offset to allow collision

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
    g_ref = KDL::Frame(q_ref, o_ref);

    // TEMPORAL CHECK UNTIL A PROPER PASSTROUGH FILTER IS APPLIED
    // the coordinates are in the left_hand_palm_link, so avoid moving topoints with z < 0 for now
    if( o_ref.z() < 0 )
    {
        eventQueue.riseEvent("/DidNotExplore");
        return TaskResult::TERMINATED();
    }


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
    // o_ref = KDL::Vector(0,0,0.3);
    // z_ref = KDL::Vector(0,0,1);
    KDL::Joint touch_joint("touch_joint",
                           o_ref,
                           z_ref,
                           KDL::Joint::RotAxis);
    // KDL::Frame touch_tip = touch_joint.pose(0);
    // rotate the tip so that z points inward
    // touch_tip.M.DoRotY(3.141592);
    KDL::Segment touch_segment( "touch_segment", touch_joint, g_ref);
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

    // configure dual group and move, set solution target, and go!
    moveit::planning_interface::MoveGroup two_group("two_arms");
    two_group.setMaxVelocityScalingFactor(0.5);

    // solve IK of the dismembered with goal being the identity to close the loop
    // initial guess for attemp 0 is the current state
    // each attempt increases by the grow factor the noise added to the initial guess
    // toDO: add weights to joints, since 1 deg. is different for proximal and distal joints
    int IK_attempts = 5;
    double noise_radius = 0.0175; // approx 1 deg.
    double grow_factor = 5; // we are working in rad, so 5 is not so big

    KDL::JntArray Qii( dismembered.getNrOfJoints() );

    for( int i = 0; i < IK_attempts; ++i )
    {
        ROS_INFO("ATTEMPT #: %i, for the DualArm IK...", i);
        // create the noise vector
        std::vector<double> N( dismembered.getNrOfJoints() );
        std::generate(std::begin(N), std::end(N), gen);
        // convert to eigen
        Eigen::VectorXd En = Eigen::Map<Eigen::VectorXd>((double *)N.data(), N.size());
        // adds noise vector to initial guess
        Qii.data = Qi.data + i*grow_factor*noise_radius*En;

        cout << "Current initial guess: " << Qii.data << endl;

        // the IK solve call
        int res = ik_solver.CartToJnt(Qii, target, Qf);
        if (res == 0)
                cout << "YEI !" << endl;
        else
                cout << "BOO :(, randomizing the initial guess" << endl;

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

        // set the solution for moveit
        two_group.setJointValueTarget(solution);

        if( !(two_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
        {
            ROS_ERROR("Could not find collision free path for current IK solution. Try another one...");

            // If we ran out of IK attempts, then add one attempt to reset
            if( i = IK_attempts)
                attempts_to_reset++;

            // and check whether we go to our safe position or not
            if( attempts_to_reset > max_attempts_to_reset )
            {
                ROS_ERROR("Max collision-free IK attempts reached, moving to a safe position before continuing");
                attempts_to_reset = 0;
                two_group.setNamedTarget("two_arms_home");
                two_group.setMaxVelocityScalingFactor(0.25);
                if( !(two_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
                {
                    ROS_ERROR("An error occured during moving robots to HOME. Turnning the logic OFF...");
                    eventQueue.riseEvent("/EStop");
                    return TaskResult::TERMINATED();
                }
            }
        }
        else
        {
            // tare the force torque sensor
            ros::service::call("/probe_ft_sensor/tare", empty_srv );
            eventQueue.riseEvent("/CloseToSurface");
            return TaskResult::SUCCESS();
        }
    }

    ROS_ERROR("Couldn't find Collision-free IK solution for the current touch path. Coming back to object modelling...");
    eventQueue.riseEvent("/DidNotExplore");
    return TaskResult::TERMINATED();
}

decision_making::TaskResult touchObjectTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("Touching the object...");

    // tare the force torque sensor
    ros::service::call("/probe_ft_sensor/tare", empty_srv );

    // FOR NOW, MOVE AGAIN FORWARD TO CONTACT WITH MOVEIT
    double safety_distance = 0.05;

    // dirty hack to remove object from the attached collision object
    predicted_collider.object.operation = moveit_msgs::CollisionObject::REMOVE;
    collider_pub.publish(predicted_collider);
    predicted_collider.object.operation = moveit_msgs::CollisionObject::ADD;
    shape_msgs::Mesh empty;
    predicted_collider.object.meshes.at(0) = empty;
    collider_pub.publish(predicted_collider);

    // ros::Duration switch_timer(10.0);

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

    // working with moveit for now, configure the group
    moveit::planning_interface::MoveGroup touch_group("touch_chain");
    touch_group.setEndEffector("touch");
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseArray normal_aligned_array;

    // and read from next best path and go even like very vast trajectory with sleeps!

    // we should be at index size() - 2 at this point
    // we check if we are at the last point, then we skip the go to the next point part
    for (int p = get_next_best_path_srv.response.next_best_path.points.size() - 2; p > -1 ; --p)
    {
        // 1. Check current point
        explored_path.points.push_back( current_contact );
        std_msgs::Bool T;
        if( contact_state.status == demo_logic::ContactState::IN_CONTACT )
        {
            T.data = true;
            explored_path.isOnSurface.push_back( T );
            std_msgs::Float32 value;
            value.data = 0.0f;
            explored_path.distances.push_back( value );
            ROS_INFO("Point already on surface");

            // 1. 4. Retreat to previous point
            current_pose = touch_group.getCurrentPose(touch_group.getEndEffectorLink());
            KDL::Frame retreat_pose;
            tf::poseMsgToKDL(current_pose.pose, retreat_pose);
            retreat_pose.p = retreat_pose.p - safety_distance*retreat_pose.M.UnitZ();
            tf::poseKDLToMsg(retreat_pose, current_pose.pose);

            //geometry_msgs::PoseArray normal_aligned_array;
            //normal_aligned_array.header = current_pose.header;

            normal_aligned_targets.clear();
            normal_aligned_array.poses.clear();

            // toDo: put this in a helper function, and save in memory what is constant
            for(int i = 0; i < n_div; ++i)
            {
                double current_angle = ((double)i/(double)n_div)*6.283185307179586; //2*pi
                KDL::Rotation current_orientation;
                current_orientation.DoRotZ(current_angle);
                KDL::Frame current_frame (current_orientation);
                current_frame = retreat_pose*current_frame;

                // convert to geometry msg
                geometry_msgs::PoseStamped current_target;
                tf::poseKDLToMsg(current_frame, current_target.pose);
                current_target.header = current_pose.header;
                normal_aligned_targets.push_back(current_target);

                // only needed for visual debug
                normal_aligned_array.poses.push_back(current_target.pose);
            }

            // only needed for visual debug
            pose_array_pub.publish(normal_aligned_array);

            touch_group.setPoseTargets(normal_aligned_targets, touch_group.getEndEffectorLink());
            touch_group.setMaxVelocityScalingFactor( 0.1 ); // go at 10% so we can detect the contact
            if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
            {
                ROS_ERROR("An error occured moving away surface. Coming back to object modelling...");

                // if we are here, it is because we touched something, so send that as the starting point
                get_next_best_path_srv.request.start_point = explored_path.points.back();;
                eventQueue.riseEvent("/DidNotExplore");
                return TaskResult::TERMINATED();
            }

            // 1. 5. Go to next point if any
            if( p > 0 )
            {
                geometry_msgs::PointStamped startPoint;
                geometry_msgs::Vector3Stamped startNormal;
                startPoint.header = get_next_best_path_srv.response.next_best_path.header;
                startPoint.point = get_next_best_path_srv.response.next_best_path.points.at(p).point;
                startNormal.header = get_next_best_path_srv.response.next_best_path.header;
                startNormal.vector = get_next_best_path_srv.response.next_best_path.directions.at(p).vector;

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
                g_ref = KDL::Frame(q_ref, o_ref);

                normal_aligned_targets.clear();
                normal_aligned_array.poses.clear();
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
                    current_target.header.stamp = ros::Time::now();
                    current_target.header.frame_id = processing_frame_name;
                    normal_aligned_targets.push_back(current_target);

                    // only needed for visual debug
                    normal_aligned_array.poses.push_back(current_target.pose);
                }

                // only needed for visual debug
                pose_array_pub.publish(normal_aligned_array);

                touch_group.setPoseTargets(normal_aligned_targets, touch_group.getEndEffectorLink());
                touch_group.setMaxVelocityScalingFactor( 0.5 );
                if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
                {
                    ROS_ERROR("An error occured moving to next point. Coming back to object modelling...");
                    // if we are here, it is because we wanted to retreat, but couldn't do so, then request with the last touched point
                    get_next_best_path_srv.request.start_point = explored_path.points.back();
                    eventQueue.riseEvent("/DidNotExplore");
                    return TaskResult::TERMINATED();
                }
            }

            continue;
            //eventQueue.riseEvent("/DoneExploration");
            //return TaskResult::SUCCESS();
        }
        if( contact_state.status == demo_logic::ContactState::NO_CONTACT )
        {
            T.data = false;
            explored_path.isOnSurface.push_back( T );

            // and tare the sensor at the current orienation before attempting to touch
            ros::service::call("/probe_ft_sensor/tare", empty_srv );
        }

        // 2. Configure the touch pose and go
        current_pose = touch_group.getCurrentPose(touch_group.getEndEffectorLink());
        KDL::Frame touch_pose;
        tf::poseMsgToKDL(current_pose.pose, touch_pose);
        touch_pose.p = touch_pose.p + safety_distance*touch_pose.M.UnitZ();

        // only needed for visual debug
        normal_aligned_array.header = current_pose.header;

        normal_aligned_targets.clear();
        for(int i = 0; i < n_div; ++i)
        {
            double current_angle = ((double)i/(double)n_div)*6.283185307179586; //2*pi
            KDL::Rotation current_orientation;
            current_orientation.DoRotZ(current_angle);
            KDL::Frame current_frame (current_orientation);
            current_frame = touch_pose*current_frame;

            // convert to geometry msg
            geometry_msgs::PoseStamped current_target;
            tf::poseKDLToMsg(current_frame, current_target.pose);
            current_target.header = current_pose.header;
            normal_aligned_targets.push_back(current_target);

            // only needed for visual debug
            normal_aligned_array.poses.push_back(current_target.pose);
        }

        // only needed for visual debug
        pose_array_pub.publish(normal_aligned_array);

        touch_group.setPoseTargets(normal_aligned_targets, touch_group.getEndEffectorLink());
        touch_group.setMaxVelocityScalingFactor( 0.1 );
        /*if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
        {
            ROS_ERROR("An error occured moving towards surface. Coming back to object modelling...");
            eventQueue.riseEvent("/DidNotExplore");
            return TaskResult::TERMINATED();
        }*/

        // Move until target or touch
        // BAD PRACTICE ALERT; BUT I NEEDED A QUICK
        // WE NEED A FUNCTION FOR THE GROUP THAT SAYS isOnTarget()
        touch_group.asyncMove();
        ros::Duration monitor_timer(0.1);
        int counter = 0;
        while( contact_state.status == demo_logic::ContactState::NO_CONTACT && counter < 100 )
        {
            monitor_timer.sleep();
            counter++;
        };
        touch_group.stop();

        // 3. Check the current touch
        explored_path.points.push_back( current_contact );
        if( contact_state.status == demo_logic::ContactState::IN_CONTACT && !explored_path.isOnSurface.at(0).data)
        {
            T.data = true;
            explored_path.isOnSurface.push_back( T );
            std_msgs::Float32 value;
            value.data = 0.05f;
            explored_path.distances.push_back( value ); // first distance of first point
            value.data = 0.0f;
            explored_path.distances.push_back( value ); // second point is on surface
        }
        if( contact_state.status == demo_logic::ContactState::NO_CONTACT && !explored_path.isOnSurface.at(0).data )
        {
            T.data = false;
            explored_path.isOnSurface.push_back( T );
            // twice political 0.03
            std_msgs::Float32 value;
            value.data = 0.03f;
            explored_path.distances.push_back( value );
            explored_path.distances.push_back( value );
        }

        // 4. Retreat to previous point
        current_pose = touch_group.getCurrentPose(touch_group.getEndEffectorLink());
        KDL::Frame retreat_pose;
        tf::poseMsgToKDL(current_pose.pose, retreat_pose);
        retreat_pose.p = retreat_pose.p - safety_distance*retreat_pose.M.UnitZ();
        tf::poseKDLToMsg(retreat_pose, current_pose.pose);

        //geometry_msgs::PoseArray normal_aligned_array;
        //normal_aligned_array.header = current_pose.header;

        normal_aligned_targets.clear();
        normal_aligned_array.poses.clear();

        // toDo: put this in a helper function, and save in memory what is constant
        for(int i = 0; i < n_div; ++i)
        {
            double current_angle = ((double)i/(double)n_div)*6.283185307179586; //2*pi
            KDL::Rotation current_orientation;
            current_orientation.DoRotZ(current_angle);
            KDL::Frame current_frame (current_orientation);
            current_frame = retreat_pose*current_frame;

            // convert to geometry msg
            geometry_msgs::PoseStamped current_target;
            tf::poseKDLToMsg(current_frame, current_target.pose);
            current_target.header = current_pose.header;
            normal_aligned_targets.push_back(current_target);

            // only needed for visual debug
            normal_aligned_array.poses.push_back(current_target.pose);
        }

        // only needed for visual debug
        pose_array_pub.publish(normal_aligned_array);

        touch_group.setPoseTargets(normal_aligned_targets, touch_group.getEndEffectorLink());
        touch_group.setMaxVelocityScalingFactor(0.7);
        if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
        {
            ROS_ERROR("An error occured moving away surface. Coming back to object modelling...");
            // if we are here, it is because we wanted to retreat, but couldn't do so, then request with the last touched point
            get_next_best_path_srv.request.start_point = explored_path.points.back();
            eventQueue.riseEvent("/DidNotExplore");
            return TaskResult::TERMINATED();
        }

        // 5. Go to next point if any
        if( p > 0 )
        {
            geometry_msgs::PointStamped startPoint;
            geometry_msgs::Vector3Stamped startNormal;
            startPoint.header = get_next_best_path_srv.response.next_best_path.header;
            startPoint.point = get_next_best_path_srv.response.next_best_path.points.at(p).point;
            startNormal.header = get_next_best_path_srv.response.next_best_path.header;
            startNormal.vector = get_next_best_path_srv.response.next_best_path.directions.at(p).vector;

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
            g_ref = KDL::Frame(q_ref, o_ref);

            normal_aligned_targets.clear();
            normal_aligned_array.poses.clear();
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
                current_target.header.stamp = ros::Time::now();
                current_target.header.frame_id = processing_frame_name;
                normal_aligned_targets.push_back(current_target);

                // only needed for visual debug
                normal_aligned_array.poses.push_back(current_target.pose);
            }

            // only needed for visual debug
            normal_aligned_array.header.frame_id = processing_frame_name;
            pose_array_pub.publish(normal_aligned_array);
            touch_group.setPoseTargets(normal_aligned_targets, touch_group.getEndEffectorLink());
            touch_group.setMaxVelocityScalingFactor(0.5);
            if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
            {
                ROS_ERROR("An error occured moving to next point. Coming back to object modelling...");
                // if we are here, it is because we wanted to retreat, but couldn't do so, then request with the last touched point
                get_next_best_path_srv.request.start_point = explored_path.points.back();
                eventQueue.riseEvent("/DidNotExplore");
                return TaskResult::TERMINATED();
            }
        }
    }

    eventQueue.riseEvent("/DoneExploration");
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
    double super_safety_distance = 0.15;

    // configure, set targets and work frames, and move the group
    moveit::planning_interface::MoveGroup touch_group("touch_chain");
    touch_group.setEndEffector("touch");
    geometry_msgs::PoseStamped current_pose = touch_group.getCurrentPose(touch_group.getEndEffectorLink());
    KDL::Frame retreat_pose;
    tf::poseMsgToKDL(current_pose.pose, retreat_pose);
    retreat_pose.p = retreat_pose.p - super_safety_distance*retreat_pose.M.UnitZ();
    tf::poseKDLToMsg(retreat_pose, current_pose.pose);

    geometry_msgs::PoseArray normal_aligned_array;
    normal_aligned_array.header = current_pose.header;

    normal_aligned_targets.clear();
    normal_aligned_array.poses.clear();
    for(int i = 0; i < n_div; ++i)
    {
        double current_angle = ((double)i/(double)n_div)*6.283185307179586; //2*pi
        KDL::Rotation current_orientation;
        current_orientation.DoRotZ(current_angle);
        KDL::Frame current_frame (current_orientation);
        current_frame = retreat_pose*current_frame;

        // convert to geometry msg
        geometry_msgs::PoseStamped current_target;
        tf::poseKDLToMsg(current_frame, current_target.pose);
        current_target.header = current_pose.header;
        normal_aligned_targets.push_back(current_target);

        // only needed for visual debug
        normal_aligned_array.poses.push_back(current_target.pose);
    }

    // only needed for visual debug
    pose_array_pub.publish(normal_aligned_array);

    touch_group.setPoseTargets(normal_aligned_targets, touch_group.getEndEffectorLink());

    if( !(touch_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured moving away surface. Coming back to object modelling...");
        //eventQueue.riseEvent("/FinishedRetreat");
        //return TaskResult::TERMINATED();
    }

    // ROS_INFO("Now, take probe to home...");

    // configure the group
    /*moveit::planning_interface::MoveGroup right_group("right_arm_probe");

    // configure the home move
    right_group.setNamedTarget("right_arm_home");

    // call the move
    if( !(right_group.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
    {
        ROS_ERROR("An error occured moving the probe to home. But exploration was succesful, retry moving away again...");
        eventQueue.riseEvent("/DoneExploration");
        return TaskResult::TERMINATED();
    }*/

    // tare the force torque sensor
    ros::service::call("/probe_ft_sensor/tare", empty_srv );
    get_next_best_path_srv.request.start_point = empty_point;

    eventQueue.riseEvent("/FinishedRetreat");

    return TaskResult::SUCCESS();
}


decision_making::TaskResult emergencyStopTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_WARN("EStop called, do something");
    return TaskResult::SUCCESS();
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
        GetObject,
        GaussianModel,
        UpdateModel,
        ExplorationStrategy,
        ApproachToSurface,
        TactileExploration,
        RetreatFromSurface,
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
                FSM_ON_EVENT("/WarmStart", FSM_NEXT(ExplorationStrategy));
            }
        }
        FSM_STATE(Home)
        {
            FSM_CALL_TASK(Home);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/Start", FSM_NEXT(GetObject));
            }
        }
        FSM_STATE(GetObject)
        {
            FSM_CALL_TASK(getObject);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ObjectReceived", FSM_NEXT(GaussianModel));
            }
        }
        FSM_STATE(GaussianModel)
        {
            FSM_CALL_TASK(createModel);

            FSM_TRANSITIONS
            {
                // FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ObjectModeled", FSM_NEXT(ExplorationStrategy));
            }
        }
        FSM_STATE(UpdateModel)
        {
            FSM_CALL_TASK(updateModel);

            FSM_TRANSITIONS
            {
                // FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ModelUpdated", FSM_NEXT(ExplorationStrategy));
            }
        }
        FSM_STATE(ExplorationStrategy)
        {
            FSM_CALL_TASK(toExploreTrajectory);

            FSM_TRANSITIONS
            {
                // FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/PolicyGenerated", FSM_NEXT(ApproachToSurface));
                FSM_ON_EVENT("/GoodObjectModel", FSM_NEXT(End));
                FSM_ON_EVENT("/RecomputeModel", FSM_NEXT(GaussianModel));
                FSM_ON_EVENT("/DidNotExplore", FSM_NEXT(ExplorationStrategy));
            }
        }
        FSM_STATE(ApproachToSurface)
        {
            // Move with position control and planning, in free space, close to the surface with:
            //    * The z-axis of tip parallel to surface normal at desired contact point
            //    * Displace according to model uncertainty along z-axis ?

            FSM_CALL_TASK(moveCloser);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/CloseToSurface", FSM_NEXT(TactileExploration));
                FSM_ON_EVENT("/DidNotExplore", FSM_NEXT(ExplorationStrategy));
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
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/DoneExploration", FSM_NEXT(RetreatFromSurface));
                FSM_ON_EVENT("/DidNotExplore", FSM_NEXT(UpdateModel));
            }
        }
        FSM_STATE(RetreatFromSurface)
        {
            // with position control and planning, away from the surface
            //    * Move away in the z-axis (normal direction) until no contact (~0N along z-axis)
            //    * With high stiffness in all direction return to home position

            FSM_CALL_TASK(moveAway);

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/FinishedRetreat", FSM_NEXT(UpdateModel));
                FSM_ON_EVENT("/DidNotExplore", FSM_NEXT(UpdateModel));
            }
        }
        FSM_STATE(End)
        {
            // FSM_CALL_TASK(returnObject)

            FSM_TRANSITIONS
            {
                // FSM_ON_EVENT("/EStop", FSM_NEXT(Off));
                FSM_ON_EVENT("/ObjectReturned", FSM_NEXT(Home));
            }
        }
    }
    FSM_END
}

// Couldn't find a cleaner way to do it, perhaps grepping some parameter for
// "left_hand" or something
void addAllowedCollisionLinks()
{
    predicted_collider.touch_links.push_back("left_hand_clamp");
    predicted_collider.touch_links.push_back( "left_hand_index_distal_link" );
    predicted_collider.touch_links.push_back( "left_hand_index_knuckle_link" );
    predicted_collider.touch_links.push_back( "left_hand_index_middle_link" );
    predicted_collider.touch_links.push_back( "left_hand_index_proximal_link" );
    predicted_collider.touch_links.push_back( "left_hand_kuka_coupler_bottom" );
    predicted_collider.touch_links.push_back( "left_hand_little_distal_link" );
    predicted_collider.touch_links.push_back( "left_hand_little_knuckle_link" );
    predicted_collider.touch_links.push_back( "left_hand_little_middle_link" );
    predicted_collider.touch_links.push_back( "left_hand_little_proximal_link" );
    predicted_collider.touch_links.push_back( "left_hand_middle_distal_link" );
    predicted_collider.touch_links.push_back( "left_hand_middle_knuckle_link" );
    predicted_collider.touch_links.push_back( "left_hand_middle_middle_link" );
    predicted_collider.touch_links.push_back( "left_hand_middle_proximal_link" );
    predicted_collider.touch_links.push_back( "left_hand_palm_link" );
    predicted_collider.touch_links.push_back( "left_hand_ring_distal_link" );
    predicted_collider.touch_links.push_back( "left_hand_ring_knuckle_link" );
    predicted_collider.touch_links.push_back( "left_hand_ring_middle_link" );
    predicted_collider.touch_links.push_back( "left_hand_ring_proximal_link" );
    predicted_collider.touch_links.push_back( "left_hand_softhand_base" );
    predicted_collider.touch_links.push_back( "left_hand_thumb_distal_link" );
    predicted_collider.touch_links.push_back( "left_hand_thumb_knuckle_link" );
    predicted_collider.touch_links.push_back( "left_hand_thumb_proximal_link" );
    predicted_collider.touch_links.push_back( "left_arm_7_link" );
    predicted_collider.touch_links.push_back( "left_arm_6_link" );
    predicted_collider.touch_links.push_back( "left_arm_5_link" );
    predicted_collider.touch_links.push_back( "box1" );
    predicted_collider.touch_links.push_back( "box2" );
    predicted_collider.touch_links.push_back( "box3" );
    predicted_collider.touch_links.push_back( "box4left1" );
    predicted_collider.touch_links.push_back( "box4left2" );
    predicted_collider.touch_links.push_back( "box4left3" );
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

    // set algorithm parameters (manually tuned for normalized training sets)
    nodeHandle.param("goal", global_variance, 0.05);
    // expand the goal for initial guesses
    expanded_variance = 3*global_variance;
    // and set the reduction rate for convergence
    reduction_rate = 0.6;

    // sound client
    talker = nodeHandle.advertise<sound_play::SoundRequest>("/robotsound", 5);

    // only for visual debug
    pose_array_pub = nodeHandle.advertise<geometry_msgs::PoseArray>("/normal_aligned_targets", 16, true);

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

    // configure voice
    voice.command = sound_play::SoundRequest::PLAY_ONCE;
    voice.sound = sound_play::SoundRequest::SAY;

    // configure the predicted collider
    nodeHandle.param<std::string>("/processing_frame", processing_frame_name, "/left_hand_palm_link");

    predicted_collider.link_name = "left_hand_palm_link";
    predicted_collider.object.header.frame_id = "left_hand_palm_link";
    I.orientation.w = 1.0;
    predicted_collider.object.mesh_poses.push_back( I );
    predicted_collider.object.id = "predicted_shape";
    predicted_collider.object.meshes.resize( 1 );
    // helper function to have a shorter main
    addAllowedCollisionLinks();
    collider_pub = nodeHandle.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1, true);

    // configure contacter
    contact_sub = nodeHandle.subscribe("/tactile_info", 100, registerContactInfo);
    contact_state.status = demo_logic::ContactState::NO_CONTACT;
    tf_listener = make_shared<tf::TransformListener>();

    // 2: Register tasks
    LocalTasks::registrate("Home", homeTask);
    LocalTasks::registrate("getObject", getObjectTask);
    LocalTasks::registrate("createModel", createModelTask);
    LocalTasks::registrate("toExploreTrajectory", generateTrajectoryTask);
    LocalTasks::registrate("updateModel", updateModelTask);
    LocalTasks::registrate("moveCloser", moveCloserTask);
    LocalTasks::registrate("touchIt", touchObjectTask);
    LocalTasks::registrate("moveAway", moveAwayTask);
    LocalTasks::registrate("emergencyStop", emergencyStopTask);

    // 3: Go!
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting DR54 demo...");
    eventQueue.async_spin();
    FsmDR54Logic(NULL, &eventQueue);

    spinner.stop();

    return 0;
}


