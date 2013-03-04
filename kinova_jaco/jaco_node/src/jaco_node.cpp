#include <jaco_api/jaco_api.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <string>
#include <jaco_node/RetractJacoArm.h>
#include <jaco_node/JointPose.h>
#include <jaco_node/FingerPose.h>
#include <LinearMath/btMatrix3x3.h>
#include <boost/array.hpp>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <angles/angles.h>

namespace jaco_node
{
    void quaternionToRPY(const btQuaternion& q, double& r, double& p, double& y)
    {
        //Using Bullet Library        
        btMatrix3x3 m(q);
        m.getRPY(r,p,y);
    }

    /// \brief Kinova Jaco arm control node.
    ///
    /// Parameters:
    ///  - jaco_key: Your API key.
    ///  - period: Update period, in s. Default: 0.1s.
    class JacoNode
    {
    public:
        typedef actionlib::ActionServer<
            control_msgs::FollowJointTrajectoryAction> FJTAS;
        typedef FJTAS::GoalHandle GoalHandleFollow;

        /// \brief Constructor.
        ///
        /// Tries to initialise the API. 
        /// If it doesn't succeed, the node will still run, but won't publish or
        /// listen to any topic.
        JacoNode(): 
            action_server_follow_(
                n_, 
                "jaco_kinematic_chain_controller/follow_joint_trajectory",
                boost::bind(&JacoNode::fjtaGoalCB, this, _1),
                boost::bind(&JacoNode::fjtaCancelCB, this, _1),
                false),
            arm_(0)
        {
            ros::NodeHandle np("~");
            std::string key;
            np.getParam("jaco_key", key);
            try
            {
                arm_ = new jaco_api::JacoArm(key.c_str());
            }
            catch (jaco_api::JacoArmException e)
            {
                ROS_ERROR("Jaco API exception: %s", e.what());
                return;
            }

			//JOINTS
            js_.name.resize(jaco_api::JOINT_COUNT);
            js_.position.resize(jaco_api::JOINT_COUNT);
            js_.velocity.resize(jaco_api::JOINT_COUNT);
            js_.effort.resize(jaco_api::JOINT_COUNT);
            pub_js_ = n_.advertise<sensor_msgs::JointState>("joint_states", 10);
            
 
            np.param("base_frame", base_frame_, std::string("/jaco_base_link"));
            hp_.header.frame_id = base_frame_;
            pub_hp_ = n_.advertise<geometry_msgs::PoseStamped>("hand_pose", 10);

            pub_cur_goal_ = 
                np.advertise<geometry_msgs::PoseStamped>("cur_goal", 10);

            sub_rel_cart_ = n_.subscribe("cmd_rel_cart", 10,
                &JacoNode::cmdRelCartCB, this);
            sub_hand_goal_ = n_.subscribe("hand_goal", 10,
                &JacoNode::handGoalCB, this);
            sub_abs_joint_ = n_.subscribe("cmd_abs_joint", 10,
                &JacoNode::cmdAbsJointCB, this);
            sub_abs_finger_ = n_.subscribe("cmd_abs_finger", 10,
                &JacoNode::cmdAbsFingerCB, this);

            double period;
            np.param("period", period, 0.1); // Update period.
            timer_ = n_.createTimer(ros::Duration(period),
                &JacoNode::timerCB, this);

            //Retract Service
            ser_retract_ = n_.advertiseService("jaco_node/retract", 
                &jaco_node::JacoNode::retract,this);

            // Joint trajectory action server.
            action_server_follow_.start();

        }

        ~JacoNode()
        {
            delete arm_;
        }

        void timerCB(const ros::TimerEvent&)
        {
            using namespace jaco_api;

            arm_->update();
            const JacoArmState& s = arm_->state();
            
            lastState_ = s;
            
            
            // Publish joint states & finger states.
            js_.header.stamp = ros::Time::now();
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                js_.name[i] = JOINT_NAMES[i];
                js_.position[i] = 
                    angles::normalize_angle(s.joints_[i].angle_);
                js_.velocity[i] = s.joints_[i].velocity_;
                js_.effort[i] = s.joints_[i].effort_;
            }
            pub_js_.publish(js_);
            
            // Publish hand position.
            hp_.header.stamp = js_.header.stamp;
            hp_.pose.position.x = s.hand_position_[0];
            hp_.pose.position.y = s.hand_position_[1];
            hp_.pose.position.z = s.hand_position_[2];
            
            // Keep the orientation in its original notation.
            roll_ = s.hand_orientation_[0];
            pitch_ = s.hand_orientation_[1];
            yaw_ = s.hand_orientation_[2];
            
            btQuaternion o = btQuaternion::getIdentity();
            
            // Perform intrinsic X-Y-Z Euler rotation with multiple quaternions.
            // TODO: Eventually replace this by something more efficient.
            btQuaternion tq;
            // Z around first joint.
            tq.setRotation(btVector3(0,0,1), js_.position[0]); 
            o *= tq;
            tq.setRotation(btVector3(1,0,0), s.hand_orientation_[0]); // X
            o *= tq;
            tq.setRotation(btVector3(0,1,0), s.hand_orientation_[1]); // Y
            o *= tq;
            tq.setRotation(btVector3(0,0,1), s.hand_orientation_[2]); // Z
            o *= tq;
                       
            lastOrientation_ = o;
                
            tf::quaternionTFToMsg(o, hp_.pose.orientation);
            pub_hp_.publish(hp_);
            
            //ROS_DEBUG("Hand pose RPY: %f, %f, %f", s.hand_orientation_[0], 
            //    s.hand_orientation_[1], s.hand_orientation_[2]);

        }

        void cmdRelCartCB(const geometry_msgs::Twist::ConstPtr& msg)
        {
            // Linear only for now.
            arm_->sendRelCartCommand(
                msg->linear.x,
                msg->linear.y,
                msg->linear.z);
        }

        void cmdAbsJointCB(const jaco_node::JointPose::ConstPtr& msg)
        {
            double joints[6];
            std::copy(msg->joints.begin(), msg->joints.end(), joints);
            arm_->sendAbsJointCommand(joints);
        }

        void cmdAbsFingerCB(const jaco_node::FingerPose::ConstPtr& msg)
        {
            double fingers[3];
            std::copy(msg->fingers.begin(), msg->fingers.end(), fingers);
            arm_->sendAbsFingerCommand(fingers);
        }

        void handGoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            // Wait for a transform between the incoming message and the arm.
            std::string err_msg;
            if (!tf_.waitForTransform(base_frame_, msg->header.frame_id, 
                msg->header.stamp, ros::Duration(0.5), ros::Duration(0.1), 
                &err_msg))
            {
                ROS_ERROR("Couldn't transform between %s and %s. Reason: %s",
                    msg->header.frame_id.c_str(), base_frame_.c_str(),
                    err_msg.c_str());
                return;
            }

            tf::Stamped<tf::Pose> pose_org, pose;
            tf::poseStampedMsgToTF(*msg, pose_org);
            tf_.transformPose(base_frame_, pose_org, pose);
            btMatrix3x3 goal_rot = pose.getBasis();

            double roll, pitch, yaw;
            // 1. The orientation is relative to the second joint - get the
            //    reverse rotation.
            btMatrix3x3 j1_rot = btMatrix3x3::getIdentity();
            // Reverse angle since we want to get the pose relative to the 
            // first joint.
            j1_rot.setRPY(0, 0, - js_.position[0]); 
            // Apply the rotation.
            goal_rot = j1_rot * goal_rot;

            // 2. Roll - Find the angle between Z and Z' (Zg projected on the 
            //    YZ plane).
            btVector3 xi(1, 0, 0);
            btVector3 zi(0, 0, 1);
            btVector3 zg = goal_rot.getColumn(2);
            ROS_DEBUG("Goal Z: (%f, %f, %f)", zg.x(), zg.y(), zg.z()); 
            btVector3 zp(0, zg.y(), zg.z());
            zp.normalize();
            roll = zi.angle(zp);
            if (zi.cross(zp).dot(xi) < 0)
                roll *= -1;
            
            // 3. Pitch - Find the angle between Z' and Zg (around Y').
            btVector3 yp(0, cos(roll), sin(roll)); 
            pitch = zp.angle(zg);
            if (zp.cross(zg).dot(yp) < 0)
                pitch *= -1;

            // 4. Yaw - Find the angle between Y' and Xg (around Z'').
            btVector3 yg = goal_rot.getColumn(1);
            // Rotated Y by roll around X.
            ROS_DEBUG("YP: (%f, %f, %f)", yp.x(), yp.y(), yp.z()); 
            yaw = yp.angle(yg);
            if (yp.cross(yg).dot(zg) < 0)
                yaw *= -1;

            ROS_DEBUG("Pose RPY: %f, %f, %f", roll, pitch, yaw);

            arm_->goToPose(
                pose.getOrigin().x(),
                pose.getOrigin().y(),
                pose.getOrigin().z(),
                roll,
                pitch,
                yaw);

            if (pub_cur_goal_.getNumSubscribers())
            {
                // Publish the current goal for diagnostic purposes.
                geometry_msgs::PoseStamped cur_goal;
                tf::poseStampedTFToMsg(pose, cur_goal);
                pub_cur_goal_.publish(cur_goal);
            }

        }

        bool retract(jaco_node::RetractJacoArm::Request &req, 
            jaco_node::RetractJacoArm::Response &res )
        {

            ROS_INFO("Retracting Jaco Arm");
            arm_->retract();
            return true;
        }

        void fjtaGoalCB(GoalHandleFollow gh)
        {
            const trajectory_msgs::JointTrajectory& org_traj = 
                gh.getGoal()->trajectory;

            if (sendJointTraj(org_traj))
                gh.setAccepted();
            else
                gh.setRejected();
        }

        void fjtaCancelCB(GoalHandleFollow gh)
        {
            // TODO: Figure out a way to stop the arm in the middle of a 
            // trajectory.
        }
    private:
        /// \brief Convert and send a joint space trajectory.
        ///
        /// NOTE: In its current state, doesn't consider time or speed 
        /// indications.
        ///
        /// \return True if the conversion succeeded.
        bool sendJointTraj(const trajectory_msgs::JointTrajectory& traj)
        {
            // We assume the incoming joint trajectory is in the correct order
            // if it contains enough values.
            if (traj.joint_names.size() != 6)
            {
                ROS_ERROR("Incoming joint trajectory action doesn't contain "
                    "enough joints.");
                return false;
            }

            // Convert the incoming joint trajectory into Jaco's format, 
            // ignoring speed and time values for now.
            typedef std::vector<trajectory_msgs::JointTrajectoryPoint> Points;
            typedef Points::const_iterator it;
            const Points& points = traj.points;
            for (it i = points.begin(); i != points.end(); ++i)
            {
                typedef boost::array<double, 6> NewTrajPoint;
                NewTrajPoint p;
                std::copy(i->positions.begin(), i->positions.end(), p.begin());
                // 2. Send each trajectory point to queue them in the DSP.
                arm_->sendAbsJointCommand(p.elems);
            }

            return true;
        }

        ros::NodeHandle n_;

        ros::Publisher pub_js_;
        ros::Publisher pub_hp_;
        ros::Publisher pub_cur_goal_;
        ros::Subscriber sub_rel_cart_;
        ros::Subscriber sub_hand_goal_;
        ros::Subscriber sub_abs_joint_;
        ros::Subscriber sub_abs_finger_;
        ros::ServiceServer ser_retract_;
        ros::Timer timer_;
        tf::TransformListener tf_;

        FJTAS action_server_follow_;
        
        sensor_msgs::JointState js_;
        std::string base_frame_;
        geometry_msgs::PoseStamped hp_;
        jaco_api::JacoArm* arm_;
        jaco_api::JacoArmState lastState_; 
        tf::Quaternion lastOrientation_;
        double roll_, pitch_, yaw_;
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaco_node");

    jaco_node::JacoNode n;
    ros::spin();
}

