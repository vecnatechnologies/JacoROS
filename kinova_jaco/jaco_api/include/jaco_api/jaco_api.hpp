#ifndef JACO_API_HPP
#define JACO_API_HPP

#include <string>
#include <exception>

namespace jaco_api
{
    class JacoArmImpl;

    static const std::string JOINT_NAMES[] =
    {
        "jaco_shoulder_yaw_joint",
        "jaco_shoulder_pitch_joint",
        "jaco_elbow_pitch_joint",
        "jaco_elbow_roll_joint",
        "jaco_wrist_roll_joint",
        "jaco_hand_roll_joint",
        "jaco_finger_1_joint",
        "jaco_finger_2_joint",
        "jaco_finger_3_joint"
    };
    
    static const size_t JOINT_COUNT = 9;


    /// \brief The state of a single Jaco joint.
    struct JacoJointState
    {
        double angle_;
        double velocity_;
        double effort_;
    };

    /// \brief The state of the arm.
    ///
    /// See the C# wrapper code for fields organization.
    struct JacoArmState
    {
        JacoJointState joints_[JOINT_COUNT];
        double hand_position_[3];
        double hand_orientation_[3]; // Euler angles XYZ.
        
    };

    class JacoArmException: public std::exception
    {
    public:
        JacoArmException(const char* msg = "Undefined exception"): msg_(msg)
        {}

        virtual const char* what() const throw() { return msg_; }

    private:
        const char* msg_;

    };

    /// \brief A C++ wrapper class for the Kinova Jaco API.
    class JacoArm
    {
    public:
        /// \brief Constructor.
        ///
        /// Automatically connects to the first available arm through USB.
        /// Places the arm in the READY position.
        /// Note that the initialization procedure can take a few seconds.
        /// \param pw API password.
        JacoArm(const char* pw) throw(JacoArmException);

        /// \brief Destructor.
        ///
        /// Automatically releases any resources from the API.
        ~JacoArm();

        /// \brief Returns if the API has been correctly initialized.
        bool isEnabled() const;

        /// \brief Returns a reference to the current state of the arm.
        const JacoArmState& state() const;

        /// \brief Sync the state and commands of the arm.
        void update();

	/// \brief Retract arm
	void retract();

        /// \brief Sends a relative, cartesian command to the arm.
        void sendRelCartCommand(double x, double y, double z);

        /// \brief Sends an absolute, joint space command to the arm (rads).
        void sendAbsJointCommand(double joints[6]);
        
        /// \brief Sends an absolute, finger command to the arm (rads).
        void sendAbsFingerCommand(double fingers[3]);

        /// \brief Goes to a cartesian pose.
        ///
        /// \param x Origin in x.
        /// \param y Origin in y.
        /// \param z Origin in z.
        /// \param e_x Euler angle in x.
        /// \param e_y Euler angle in y.
        /// \param e_z Euler angle in z.
        void goToPose(double x, double y, double z, 
            double e_x, double e_y, double e_z);

    private:
        JacoArmImpl* impl_;

    };

}

#endif

