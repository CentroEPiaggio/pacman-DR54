
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

volatile bool topicCtrl;

ros::Subscriber topicSub;

/*EventQueue* mainEventQueue;
struct MainEventQueue{
	MainEventQueue(){ mainEventQueue = new RosEventQueue();	}
	~MainEventQueue(){ delete mainEventQueue; }
};*/

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
			ROS_INFO("Calling segmentation service...");

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

int main(int argc, char** argv){

	ros::init(argc, argv, "DR54_logic");
	ros_decision_making_init(argc, argv);

	// 2: 1 for the FSM and 1 for the EventQueue
	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting DR54 demo...");
	FsmDR54Logic(NULL, new RosEventQueue());

	spinner.stop();

	return 0;
}


