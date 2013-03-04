#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <jaco_node/RetractJacoArm.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/utils.h>

using namespace visualization_msgs;


namespace jaco_interactive_markers
{
    class JacoMarkers
	{
		public:

		JacoMarkers()
			: record_(false),
            move_arm_("move_jaco_kinematic_chain", true)
		{
		
    		n.param("base_frame", base_frame_, std::string("/jaco_base_link"));
			server.reset( new interactive_markers::InteractiveMarkerServer("jaco_interactive_markers","",false) );

            //Menu handler
            interactive_markers::MenuHandler::FeedbackCallback f;
            f = boost::bind(&jaco_interactive_markers::JacoMarkers::menuFeedback, this, _1);
            menu_handler.insert( "Retract", f);
            menu_handler.insert( "Set Marker to actual Hand Pose", f);
            menu_handler.insert( "Record", f);
            menu_handler.insert( "Stop Record", f);
            menu_handler.insert( "Clear Record", f);
            menu_handler.insert( "Playback", f);
            menu_handler.insert( "Send to arm_navigation", f);
    
            //Markers
            make6DofMarker( true );
            makeHandMarker();
            makeMenuMarker();

            ros::Duration(0.1).sleep();

            server->applyChanges();
            
            //Hand goal publisher
            posePublisher = n.advertise<geometry_msgs::PoseStamped>("/hand_goal", 5);
            
            
            //Hand pose subscriber
            handPoseSubscriber_ = n.subscribe("/hand_pose",5,&jaco_interactive_markers::JacoMarkers::handPoseReceived,this);
            
            
		}

		~JacoMarkers()
		{
			server.reset();
		}

        void handPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
        
            ROS_DEBUG("handPoseFeedback received");
            handPose_ = *msg;
            
             InteractiveMarker int_marker;
            if (server->get("hand_pose",int_marker))
            {
                                       
                //Update label
                std::ostringstream label;
                label   <<"Jaco Hand Pose"
                    << "\nposition = "
                    << handPose_.pose.position.x
                    << ", " << handPose_.pose.position.y
                    << ", " << handPose_.pose.position.z
                    << "\norientation = "
                    << handPose_.pose.orientation.w
                    << ", " << handPose_.pose.orientation.x
                    << ", " << handPose_.pose.orientation.y
                    << ", " << handPose_.pose.orientation.z;
            
                int_marker.description = label.str();
                //This will change the marker, without changing the callback
                server->insert(int_marker);
                
                server->applyChanges();
            }     
            
            
        }


        void setHandPoseMarker(const geometry_msgs::Pose &p)
        {
            //Let's publish the hand pose
            geometry_msgs::PoseStamped pose;

            //Set the preset pose
            pose.pose = p;

            //Get the time stamp
            pose.header.stamp = ros::Time::now();

            //Set the frame
            pose.header.frame_id = base_frame_;

            //Update GUI                     
            server->setPose("jaco_hand_pose_6dof",pose.pose,pose.header);
            server->applyChanges();

            //Publish...
            posePublisher.publish(pose);
        }


        void menuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {		  	
            std::ostringstream s;
            s << "Feedback from marker (menu)'" << feedback->marker_name << "' "
            << " / control '" << feedback->control_name << "'";
        
            switch (feedback->event_type)
            {
                case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                    ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" );
                    
                    //Retract menu
                    if (feedback->menu_entry_id == 1)
                    {

                        ROS_INFO("Calling retract");				
                        ros::ServiceClient client = n.serviceClient<jaco_node::RetractJacoArm>("/jaco_node/retract");
                        jaco_node::RetractJacoArm srv;

                        if (client.call(srv))
                        {
                            ROS_INFO("Success Calling retract");
                        }
                    }
                    else if (feedback->menu_entry_id == 2)
                    {
                        ROS_INFO("Set Marker to actual hand pose");
                        //setHandPoseMarker(geometry_msgs::Pose());                                                                           
                        setHandPoseMarker(handPose_.pose);
                    } 
                    else if (feedback->menu_entry_id == 3)
                    {
                        ROS_INFO("Record");
                        record_ = true;
                                                
                    }                     
                    else if (feedback->menu_entry_id == 4)
                    {
                        ROS_INFO("Stop Record");
                        record_ = false;
                                         
                    } 
                    else if (feedback->menu_entry_id == 5)
                    {
                        ROS_INFO("Clear Record");
                        recordList_.clear();
                                          
                    } 
                    else if (feedback->menu_entry_id == 6)
                    {
                        ROS_INFO("Playback size : %i",(int) recordList_.size());
                        
                        for (std::list<geometry_msgs::PoseStamped>::iterator iter = recordList_.begin(); iter != recordList_.end(); iter++)
                        {
	                     	//Update time
	                     	(*iter).header.stamp = ros::Time::now();
	                     	
	                     	//Publish pose
	                        posePublisher.publish(*iter);
                        }

                    } 
                    else if (feedback->menu_entry_id == 7) // send to arm_navigation
                    {
                        // Create MoveArm goal based on current pose, send it.  
                        arm_navigation_msgs::MoveArmGoal goal;
                        goal.motion_plan_request.group_name = "jaco_kinematic_chain";
                        goal.motion_plan_request.num_planning_attempts = 1;
                        goal.motion_plan_request.planner_id = std::string("");
                        goal.planner_service_name = std::string("ompl_planning/plan_kinmatic_path");
                        goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

                        arm_navigation_msgs::SimplePoseConstraint desired_pose;
                        desired_pose.header.frame_id = base_frame_;
                        desired_pose.link_name = "jaco_hand_link";
                        desired_pose.pose = feedback->pose;

                        desired_pose.absolute_position_tolerance.x = 0.02;
                        desired_pose.absolute_position_tolerance.y = 0.02;
                        desired_pose.absolute_position_tolerance.z = 0.02;

                        desired_pose.absolute_roll_tolerance = 0.04;
                        desired_pose.absolute_pitch_tolerance = 0.04;
                        desired_pose.absolute_yaw_tolerance = 0.04;

                        arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);
                        move_arm_.sendGoal(goal);
                        // NOTE: We're not interested in the success of the action, yet.
                    }


                break;
                
                default:
                break;
            
            }
        
        
        }
    



		void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
		{
		  	std::ostringstream s;
            s << "Feedback from marker '" << feedback->marker_name << "' "
            << " / control '" << feedback->control_name << "'";

            switch ( feedback->event_type )
            {
                case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
                    ROS_INFO_STREAM( s.str() << ": button click" );
                break;

                case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                    ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" );
                break;

			    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
                    ROS_INFO_STREAM( s.str() << ": pose changed"
                    << "\nposition = "
                    << feedback->pose.position.x
                    << ", " << feedback->pose.position.y
                    << ", " << feedback->pose.position.z
                    << "\norientation = "
                    << feedback->pose.orientation.w
                    << ", " << feedback->pose.orientation.x
                    << ", " << feedback->pose.orientation.y
                    << ", " << feedback->pose.orientation.z
                    << "\nframe: " << feedback->header.frame_id
                    << " time: " << feedback->header.stamp.sec << "sec, "
                    << feedback->header.stamp.nsec << " nsec" );
                                               
		        break;

			    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
			        ROS_INFO_STREAM( s.str() << ": mouse down." );
		        break;

			    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
                    ROS_INFO_STREAM( s.str() << ": mouse up." );

                    if (feedback->marker_name == "jaco_hand_pose_6dof")
                    {
                        //Let's publish the hand pose
                        geometry_msgs::PoseStamped pose;

                        //Get the pose
                        pose.pose = feedback->pose;

                        //Get the time stamp
                        pose.header.stamp = ros::Time::now();

                        //Set the frame
                        pose.header.frame_id = base_frame_;

                        //Publish...
                        if (record_)
                        {
	                        //Adding pose
	                        recordList_.push_back(pose);
	                        
	                        
	                        ROS_INFO("Adding post to record list size = %i",(int) recordList_.size());
	                        
	                        //Publishing pose	                        
                        	posePublisher.publish(pose); 
                        }
                        else
                        {
	                        //Publishing pose	                        
                        	posePublisher.publish(pose);    
                        }
                        
                        
                        //Update label
                        std::ostringstream label;
                        label   <<"Jaco 6-DOF Hand Goal"
                            << "\nposition = "
                            << feedback->pose.position.x
                            << ", " << feedback->pose.position.y
                            << ", " << feedback->pose.position.z
                            << "\norientation = "
                            << feedback->pose.orientation.w
                            << ", " << feedback->pose.orientation.x
                            << ", " << feedback->pose.orientation.y
                            << ", " << feedback->pose.orientation.z
                            << " recording : "<<record_;
                            
                        InteractiveMarker int_marker;
                        if (server->get(feedback->marker_name,int_marker))
                        {
                            int_marker.description = label.str();
                            //This will change the marker, without changing the callback
                            server->insert(int_marker);
                        }     
                                                                                            
                        
                    }
		        break;
			  }

	        server->applyChanges();
        }


	    Marker makeBox( InteractiveMarker &msg )
	    {
            Marker marker;

            marker.type = Marker::CUBE;
            marker.scale.x = msg.scale * 0.45;
            marker.scale.y = msg.scale * 0.45;
            marker.scale.z = msg.scale * 0.45;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 0.5;

            return marker;
	    }

        InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
        {
            InteractiveMarkerControl control;
            control.always_visible = true;
            control.markers.push_back( makeBox(msg) );
            msg.controls.push_back( control );

            return msg.controls.back();
        }


        void saveMarker( InteractiveMarker int_marker )
        {
            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&jaco_interactive_markers::JacoMarkers::processFeedback, this, _1));
        }

        void makeHandMarker()
        {
            InteractiveMarker int_marker;
            int_marker.header.frame_id = base_frame_;
            int_marker.pose.position.z = -1;         
            int_marker.scale = 0.25;

            int_marker.name = "hand_pose";
            int_marker.description = "Not updated yet";
            makeBoxControl(int_marker);
/*
            InteractiveMarkerControl control;

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);

            control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
            control.always_visible = true;
            control.markers.push_back( makeBox(int_marker) );
            int_marker.controls.push_back(control);
*/

            server->insert(int_marker);

            //CALLBACK
            server->setCallback(int_marker.name, boost::bind(&jaco_interactive_markers::JacoMarkers::processFeedback, this, _1));
        }


        void make6DofMarker( bool fixed )
        {
            InteractiveMarker int_marker;
            int_marker.header.frame_id = base_frame_;
            int_marker.pose.position.y = 0;
            int_marker.pose.position.x = 0;
            int_marker.scale = 0.25;

            int_marker.name = "jaco_hand_pose_6dof";
            int_marker.description = "Jaco 6-DOF Hand Goal";

            // insert a box
            makeBoxControl(int_marker);

            InteractiveMarkerControl control;
/*
            if ( fixed )
            {
                int_marker.name += "_fixed";
                int_marker.description += "\n(fixed orientation)";
                control.orientation_mode = InteractiveMarkerControl::FIXED;
            }
*/

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_x";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_y";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            
            
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            server->insert(int_marker);

            //CALLBACK
            server->setCallback(int_marker.name, boost::bind(&jaco_interactive_markers::JacoMarkers::processFeedback, this, _1));
        }

        void makeMenuMarker()
        {
            InteractiveMarker int_marker;
            int_marker.header.frame_id = base_frame_;
            int_marker.pose.position.z = -0.30;
            int_marker.scale = 0.25;

            int_marker.name = "jaco_menu";
            int_marker.description = "Jaco Menu\n(Right Click)";

            InteractiveMarkerControl control;

            //make one control using default visuals
            control.interaction_mode = InteractiveMarkerControl::MENU;
            control.description="Commands";
            control.name = "jaco_menu_control";
            int_marker.controls.push_back(control);

            // make one control showing a box
            Marker marker = makeBox( int_marker );
            control.markers.push_back( marker );
            control.always_visible = true;
            int_marker.controls.push_back(control);

            server->insert(int_marker);
            //CALLBACK
            server->setCallback(int_marker.name, boost::bind(&jaco_interactive_markers::JacoMarkers::processFeedback, this, _1));
            menu_handler.apply( *server, int_marker.name );
        }


        protected:

        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
        interactive_markers::MenuHandler menu_handler;
        ros::Publisher posePublisher;
        ros::Subscriber handPoseSubscriber_;
        geometry_msgs::PoseStamped handPose_;
        ros::NodeHandle n;
        std::string base_frame_;
        bool record_;
        std::list<geometry_msgs::PoseStamped> recordList_;

        actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;

	};//class
}//namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaco_interactive_markers");
    jaco_interactive_markers::JacoMarkers markers;
    ros::spin();
    return 0;
}
