// standard ros headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <demo_logic/GetPathLog.h>

// local headers
#include <gp_regression/Path.h>
#include <demo_logic/ContactState.h>
#include <intrinsic_tactile_toolbox/TactileInfo.h>

// keeping the global variables style from demo logic
diagnostic_msgs::DiagnosticArray demo_state;
demo_logic::ContactState contact_state;
geometry_msgs::PointStamped current_point;
geometry_msgs::WrenchStamped current_force;


std::string processing_frame;

ros::Publisher pub_contact_state_;
// ros::Publisher pub_contact_log_;
ros::Publisher pub_stop_demo_;

gp_regression::Path path_log;

// the only state we are supposed to be touching
std::string allowed_state ;

std::string state;

void isContactExpected(const geometry_msgs::WrenchStamped::ConstPtr &wrench_msg)
{
        std::cout << "Is contact expected callback" << std::endl;
        // std::string state = demo_state.status.at(0).values.at(1).value;

        double contact_force_mag = wrench_msg->wrench.force.x*wrench_msg->wrench.force.x +
                                   wrench_msg->wrench.force.y*wrench_msg->wrench.force.y +
                                   wrench_msg->wrench.force.z;
        contact_force_mag = std::sqrt(contact_force_mag);

        std::cout << "contact_force_mag: " << contact_force_mag << std::endl;

        // for logging and estopping it is the same
        double contact_force_threshold = 0.02;

        if( contact_force_mag < contact_force_threshold )
        {
                contact_state.status = demo_logic::ContactState::NO_CONTACT;
        }
        else if( contact_force_mag > contact_force_threshold )
        {
                contact_state.status = demo_logic::ContactState::IN_CONTACT;
                if( !(state.compare(allowed_state)) )
                {
                        ROS_ERROR("Unexpected contact detected! Stopping the demo...");
                        contact_state.status = demo_logic::ContactState::UNEXPECTED_CONTACT;
                        std_msgs::String estop;
                        estop.data = std::string("EStop");
                        pub_stop_demo_.publish(estop);
                }
        }

        if( state.compare(allowed_state) )
        {
                // log safely
                path_log.points.push_back( current_point );
                // path_log.directions.push_back( tactile info ); // we need the contact frame, and take the z-axis
                std_msgs::Bool label;
                if( contact_state.status == demo_logic::ContactState::NO_CONTACT )
                {
                        label.data = false;
                        path_log.isOnSurface.push_back( label );
                }

                if ( contact_state.status == demo_logic::ContactState::IN_CONTACT )
                {
                        label.data = true;
                        path_log.isOnSurface.push_back( label );
                }
        }
        pub_contact_state_.publish(contact_state);
}

void updateDemoState(const diagnostic_msgs::DiagnosticArray::ConstPtr & demo_msg)
{
        demo_state = *demo_msg;
        std::cout << "State machine changed to: " << demo_state.status.at(0).values.at(0).value << std::endl;
}

void updateContactInfo(const geometry_msgs::PointStamped::ConstPtr &point_msg)
{
        tf::TransformListener tf_listener;
        tf_listener.transformPoint(processing_frame, *point_msg, current_point);
}

bool getPathLogCallback(demo_logic::GetPathLogRequest &req, demo_logic::GetPathLogResponse &res)
{
        if( path_log.points.empty() )
                return false;
        else
                res.path_log = path_log;
        return true;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "contact_observer_node");
        ros::NodeHandle nh;

        ros::ServiceServer path_log_srv;
        path_log_srv = nh.advertiseService("/get_path_log", getPathLogCallback);
        // do we need a clear log srv?

        pub_contact_state_ = nh.advertise<demo_logic::ContactState>("contact_state", 1);
        // pub_contact_log_ = nh.advertise<gp_regression::Path>("/path_log", 1, true);
        pub_stop_demo_ = nh.advertise<std_msgs::String> ("/decision_making/DemoDR54/events", 100);

        ros::Subscriber sub_force_msr = nh.subscribe<geometry_msgs::WrenchStamped>(nh.resolveName("contact_wrench"), 10, isContactExpected);
        ros::Subscriber sub_demo_diag = nh.subscribe<diagnostic_msgs::DiagnosticArray>(nh.resolveName("/decision_making/monitoring"), 10, updateDemoState);
        ros::Subscriber sub_contact_msr = nh.subscribe<geometry_msgs::PointStamped>(nh.resolveName("contact_point"), 10, updateContactInfo);

        allowed_state = std::string("/DR54Logic/TactileExploration/TactileExploration");

        // start with no contact
        contact_state.status = demo_logic::ContactState::NO_CONTACT;

        if (ros::param::get("processing_frame", processing_frame))
        {
                processing_frame = "/camera_rgb_optical_frame";
        }

        ros::spin();

        return 0;
}
