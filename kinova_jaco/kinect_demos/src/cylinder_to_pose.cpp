#include <geometry_msgs/PoseStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <jaco_node/FingerPose.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace kinect_demos
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    /// \brief A test node that creates a set of poses suitable
    /// for grabbing a small cylinder with the Kinova arm.
    class CylinderToPose
    {
    public:
        CylinderToPose()
        {
            ros::NodeHandle np("~");

            sub_model_ = 
                n_.subscribe("model", 10, &CylinderToPose::modelCB, this);
            sub_proj_ =
                n_.subscribe("projection", 10, &CylinderToPose::projCB, this);
            sub_hand_ =
                n_.subscribe("hand_pose", 10, &CylinderToPose::handCB, this);
            sub_js_ =
                n_.subscribe("joint_states", 10, &CylinderToPose::jsCB, this);
            pub_goal_ = 
                n_.advertise<geometry_msgs::PoseStamped>("hand_goal", 10);
            pub_fingers_ = 
                n_.advertise<jaco_node::FingerPose>("cmd_abs_finger", 10);

            srv_go_ = np.advertiseService("go", &CylinderToPose::goCB, this);
            srv_seek_ = 
                np.advertiseService("seek", &CylinderToPose::seekCB, this);

            pub_toward_ = 
                np.advertise<geometry_msgs::PoseStamped>("toward", 10);
            pub_target_ = 
                np.advertise<geometry_msgs::PoseStamped>("target", 10);
            pub_lifted_ = 
                np.advertise<geometry_msgs::PoseStamped>("lifted", 10);

            timer_ = np.createTimer(ros::Duration(0.1), 
                &CylinderToPose::timerCB, this);
        }

    private:
        void projCB(const PointCloud::ConstPtr& msg)
        {
            if (msg->points.size() < 10)
            {
                //ROS_DEBUG("Less than 10 points in projection.");
                return;
            }

            // Find out the cylinder's bounds by projecting every point
            // on the model's axis.

            double min = 1;
            double max = -1;
            BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points)
            {
                tf::Vector3 p = tf::Vector3(pt.x, pt.y, pt.z) - old_p_;
                double h = p.dot(old_cyl_);
                if (h > max)
                    max = h;
                if (h < min)
                    min = h;
            }

            cyl_height_ = old_cyl_height_ + 
                (((max - min) - old_cyl_height_) / avg_count_); 
            old_cyl_height_ = cyl_height_;

        }

        void modelCB(const pcl::ModelCoefficients::ConstPtr& msg)
        {
            if (msg->values.size() < 7)
            {
                ROS_DEBUG("Not enough coefficients to be a cylinder: %i.",
                    (int)msg->values.size());
                return;
            }

            // 1. Create the target pose.

            // Cylinder structure: {{point_on_axis}, {axis_direction}, radius}.
            // The axis direction is normalized.
            //
            // TODO: Figure out where the point is relative to the cylinder,
            // e.g the base, CoG, ...
           
            // Cylinder origin, seems to be CoG.
            tf::Vector3 p(
                msg->values[0], 
                msg->values[1], 
                msg->values[2]);

            // Cylinder vertical axis.
            tf::Vector3 cyl(
                msg->values[3],
                msg->values[4],
                msg->values[5]);

            // TODO: Make sure it points to the sky (needs a reference).
            // The test is currently baked knowing the fact that the Y axis
            // of the camera points to the ground.
            if (cyl.y() > 0)
                cyl *= -1.0;

            // Cummulative average on the position, axis and height.
            avg_count_ += 1.0;
            p = old_p_ + ((p - old_p_) / avg_count_);
            cyl = old_cyl_ + ((cyl - old_cyl_) / avg_count_);
            cyl.normalize();
            old_p_ = p;
            old_cyl_ = cyl;

            // Orientation:
            //  - X points to the thumb.
            //  - Y is perpendicular to the grasp plane.
            //  - Z is the wrist rotation axis, and should point outward
            //    the camera.

            // We generate Z to point toward the ground, so the inverse of
            // the cylinder axis.
            // We generate X with the cross product of the view axis (the
            // cylinder's position) and the Y axis so that it points to the
            // left.
            // Y axis is the cross product of Z and X.
            tf::Vector3 view = p.normalized();
            tf::Vector3 oz = -1.0 * cyl;
            tf::Vector3 ox = view.cross(oz);
            tf::Vector3 oy = oz.cross(ox);

            // !! OLD SETUP !!
            // We generate X with the cross product of the cylinder axis (Y)
            // and the view vector and should point to the left.
            // Z is then produced with the cross product of X and Y.
            // tf::Vector3 view = p.normalized();
            // tf::Vector3 oy = cyl;
            // tf::Vector3 ox = oy.cross(view).normalized();
            // tf::Vector3 oz = ox.cross(oy);
            
            // Convert this into a quaternion from the basis matrix.
            tf::Matrix3x3 basis(
                ox.x(), oy.x(), oz.x(),
                ox.y(), oy.y(), oz.y(),
                ox.z(), oy.z(), oz.z());
           
            // Convert to the pose message.
            // Target pose: half a cylinder height back from the center, minus
            // 3cm to give enough grip to the fingers.
            tf::Vector3 target_p = p - ((0.5 * cyl_height_ - 0.03) * oz);
            target_pose_.header =  msg->header;
            tf::pointTFToMsg(target_p, target_pose_.pose.position);
            tf::Quaternion q;
            basis.getRotation(q);
            tf::quaternionTFToMsg(q, target_pose_.pose.orientation);

            // 1. Create the "toward" pose (back a full cylinder height).
            tf::Vector3 toward_pos = p - (cyl_height_ * oz);
            toward_pose_.header = target_pose_.header;
            tf::pointTFToMsg(toward_pos, toward_pose_.pose.position);
            toward_pose_.pose.orientation = target_pose_.pose.orientation;

            // 2. Create the "lifted" pose (same as "toward").
            tf::Vector3 lifted_pos = toward_pos;
            lifted_pose_.header = target_pose_.header;
            tf::pointTFToMsg(lifted_pos, lifted_pose_.pose.position);
            lifted_pose_.pose.orientation = target_pose_.pose.orientation;
                
            // For diagnostic purposes:
            pub_toward_.publish(toward_pose_);
            pub_target_.publish(target_pose_);
            pub_lifted_.publish(lifted_pose_);

        }

        bool goCB(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
        {
            // Fix the poses for the state machine.
            fixed_toward_pose_ = toward_pose_;
            fixed_target_pose_ = target_pose_;
            fixed_lifted_pose_ = lifted_pose_;
            state_ = GO;

            return true;

        }

        bool seekCB(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
        {
            // Reset the cumulative average.
            avg_count_ = 1;

            return true;
        }

        void handCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            hand_pose_ = *msg;
        }

        void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
        {
            // Only keep the fingers' states.
            std::copy(&(msg->position[6]), &(msg->position[9]), &fingers_[0]);
        }

        /// \brief Returns if we're at the specified position.
        bool at(const geometry_msgs::PoseStamped& ref)
        {
            tf::Vector3 p, ref_p;

            tf::Stamped<tf::Pose> hp_tmp, hp;
            tf::poseStampedMsgToTF(hand_pose_, hp_tmp);
            try {
                tf_.transformPose(ref.header.frame_id, hp_tmp, hp);
            } catch (tf::TransformException)
            {
                ROS_WARN("Can't transform hand pose.");
                return false;
            }

            tf::pointMsgToTF(ref.pose.position, ref_p);
            double dist = (hp.getOrigin() - ref_p).length();
            ROS_DEBUG_THROTTLE(1, "dist: %f", dist);

            if (dist < 0.005) // large epsilon.
                return true;
            else
                return false;
        }

        bool handOpened()
        {
            // Only check one finger:
            if (fingers_[0] < 0.2)
                return true;
            else
                return false;
        }

        void openHand()
        {
            jaco_node::FingerPose msg;
            std::fill(msg.fingers.begin(), msg.fingers.end(), 0.0);
            pub_fingers_.publish(msg);
        }

        void closeHand()
        {
            jaco_node::FingerPose msg;
            std::fill(msg.fingers.begin(), msg.fingers.end(), 0.5);
            pub_fingers_.publish(msg);
        }

        /// \brief Publishes on pub_goal, automatically set time to now.
        void pubGoal(geometry_msgs::PoseStamped& msg)
        {
            msg.header.stamp = ros::Time::now();
            pub_goal_.publish(msg);
        }

        void timerCB(const ros::TimerEvent&)
        {
            ROS_DEBUG_THROTTLE(1, "State: %i", (int)state_);

            switch (state_)
            {
                case IDLE:
                    // Do nothing.
                break;

                case GO:
                    // Always transition to OPEN_HAND, send an open command if
                    // necessary.
                    if (!handOpened())
                    {
                        openHand();
                    }
                    state_ = OPEN_HAND;
                break;

                case OPEN_HAND:
                    if (handOpened())
                    {
                        // Transition to toward pose.
                        pubGoal(fixed_toward_pose_);
                        state_ = GO_TOWARD; 
                    }
                break;

                case GO_TOWARD:
                    if (at(fixed_toward_pose_))
                    {
                        // Transition to target pose.
                        pubGoal(fixed_target_pose_);
                        state_ = LOWER;
                    }
                break;

                case LOWER:
                    if (at(fixed_target_pose_))
                    {
                        // Transition to hand closing.
                        closeHand();
                        state_ = CLOSE_HAND;
                    }
                break;

                case CLOSE_HAND:
                    if (!handOpened())
                    {
                        // Transition to lifted pose.
                        pubGoal(fixed_lifted_pose_);
                        state_ = LIFT;
                    }
                break;

                case LIFT:
                    if (at(fixed_lifted_pose_))
                    {
                        // Transition back to idle.
                        state_ = IDLE;
                    }
                break;

                default:
                    state_ = IDLE;
                    break;

            };
        }

        ros::NodeHandle n_;
        tf::TransformListener tf_;
        ros::Subscriber sub_model_;
        ros::Subscriber sub_proj_;
        ros::Subscriber sub_hand_;
        ros::Subscriber sub_js_;
        ros::Subscriber sub_fingers_;
        ros::Publisher pub_goal_;
        ros::Publisher pub_fingers_;
        ros::ServiceServer srv_go_;
        ros::ServiceServer srv_seek_;
        ros::Timer timer_;

        ros::Publisher pub_toward_;
        ros::Publisher pub_target_;
        ros::Publisher pub_lifted_;

        geometry_msgs::PoseStamped toward_pose_;
        geometry_msgs::PoseStamped target_pose_;
        geometry_msgs::PoseStamped lifted_pose_;
        geometry_msgs::PoseStamped fixed_toward_pose_;
        geometry_msgs::PoseStamped fixed_target_pose_;
        geometry_msgs::PoseStamped fixed_lifted_pose_;
        geometry_msgs::PoseStamped hand_pose_;
        boost::array<double, 3> fingers_;

        tf::Vector3 old_p_;
        tf::Vector3 old_cyl_;
        double cyl_height_;
        double old_cyl_height_;
        double avg_count_;

        // State machine for the grasping task.
        enum State
        {   
            IDLE,       // Waiting for the go signal
            GO,         // Open the hand.
            OPEN_HAND,  // Wait for an opened hand.
            GO_TOWARD,  // Go to the toward pose.
            LOWER,      // Go to the target position.
            CLOSE_HAND, // Grasp the object, wait for a closed hand.
            LIFT        // Go to the lifted position.
        };
        State state_;
         
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cylinder_to_pose");

    kinect_demos::CylinderToPose node;

    ros::spin();

    return 0;
}

