#include "jaco_api/jaco_api.hpp"
#include <mono/jit/jit.h>
#include <mono/metadata/mono-config.h>
#include <mono/metadata/debug-helpers.h>
#include <mono/metadata/assembly.h>
#include <mono/metadata/class.h>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <glib.h>

// A symbol named API_PATH should be defined with a trailing '/' to point at
// the directory where the API DLLs can be found.
#define XSTR(S) STR(S)
#define STR(S) #S
#define API_PATH_STR XSTR(API_PATH)

namespace jaco_api 
{
    class JacoArmImpl
    {
    public:
        JacoArmImpl(const char* key)
        : domain_(0), 
            assembly_(0), 
            image_(0), 
            JacoArm_(0),             
            JacoArmCtor_(0),
            JacoArmIsEnabled_(0),
            JacoArmState_(0),
            JacoArmUpdate_(0),
            JacoArmSendRelCartCommand_(0),
            JacoArmGoToPose_(0),   
	        JacoRetract_(0),
	        JacoJointsAngleRads_(0),
	        JacoFingersAngleRads_(0),                       
            arm_(0) 
        {
            domain_ = mono_jit_init_version("jaco_ros_wrapper", "v2.0.50727");
            mono_config_parse(NULL);

            std::cerr << "Full path: " API_PATH_STR "JacoAPIWrapper.dll" <<
                std::endl;

            assembly_ =
                mono_domain_assembly_open(domain_, 
                    API_PATH_STR "JacoAPIWrapper.dll");
            if (!assembly_)
               throw JacoArmException("Cannot load assembly!");

            image_ = 
                mono_assembly_get_image(assembly_);

            JacoArm_ = 
                mono_class_from_name(image_, "JacoAPIWrapper", "JacoArm");
            if (!JacoArm_)
                throw JacoArmException("Cannot find JacoArm class!");
            
            // Enumerate methods, keep the ones we need for now.
            MonoMethod* m = 0;
            void* iter = 0;
            while ((m = mono_class_get_methods(JacoArm_, &iter)))
            {
                if (strcmp(mono_method_get_name(m), ".ctor") == 0)
                    JacoArmCtor_ = m;
                else if (strcmp(mono_method_get_name(m), "IsEnabled") == 0)
                    JacoArmIsEnabled_ = m;
                else if (strcmp(mono_method_get_name(m), "GetState") == 0)
                    JacoArmState_ = m;
                else if (strcmp(mono_method_get_name(m), "Update") == 0)
                    JacoArmUpdate_ = m;
                else if (strcmp(mono_method_get_name(m), "SendRelCartCommand") == 0)
                    JacoArmSendRelCartCommand_ = m;
                else if (strcmp(mono_method_get_name(m), "GoToPose") == 0)
                    JacoArmGoToPose_ = m;     
		        else if (strcmp(mono_method_get_name(m), "Retract") == 0)
		            JacoRetract_ = m;
		        else if (strcmp(mono_method_get_name(m), "SendJointsAngleRads") == 0)
                    JacoJointsAngleRads_ = m;
                else if (strcmp(mono_method_get_name(m), "SendFingersAngleRads") == 0)
                	JacoFingersAngleRads_ = m;
            }

            //Testing method pointers
            if (!JacoArmCtor_)
                throw JacoArmException("Cannot find JacoArm constructor!");
            if (!JacoArmIsEnabled_)
                throw JacoArmException("Cannot find IsEnabled() method!");
            if (!JacoArmState_)
                throw JacoArmException("Cannot find State() method!");
            if (!JacoArmUpdate_)
                throw JacoArmException("Cannot find Update() method!");
            if (!JacoArmSendRelCartCommand_)
                throw JacoArmException("Cannot find SendRelCartCommand method!");
            if (!JacoArmGoToPose_)
                throw JacoArmException("Cannot find GoToPose method!");
	        if (!JacoRetract_)
		        throw JacoArmException("Cannot find Retract method!");
		    if (!JacoJointsAngleRads_)
		        throw JacoArmException("Cannot find SendJointsAngleRads method!");
		    if (!JacoFingersAngleRads_)
		    	throw JacoArmException("Cannot find SendFingersAngleRads method!");

            arm_ = mono_object_new(domain_, JacoArm_); 
            // This is to keep the arm object from being garbage-collected.
            arm_handle_ = mono_gchandle_new(arm_, TRUE);

            void* args[1];
            args[0] = mono_string_new(domain_, key);
            mono_runtime_invoke(JacoArmCtor_, arm_, args, NULL);

            // Check if it initialized correctly.
            if (!isEnabled())
                throw JacoArmException("The API didn't initialized the robot "
                    "correctly!");
            // Get the initial state.
            mono_runtime_invoke(JacoArmUpdate_, arm_, NULL, NULL);
            MonoObject* mstate =
                mono_runtime_invoke(JacoArmState_, arm_, NULL, NULL);
            state_ = *((JacoArmState*)mono_object_unbox(mstate));

        }

        ~JacoArmImpl()
        {
            mono_gchandle_free(arm_handle_);
            mono_jit_cleanup(domain_);
        }
        
        bool isEnabled() const
        {
            if (!arm_)
                return false;

            MonoObject* r = 
                mono_runtime_invoke(JacoArmIsEnabled_, arm_, NULL, NULL);
            return (* (MonoBoolean*)mono_object_unbox(r));
        }

        const JacoArmState& state() const
        {
            return state_;
        }

        void update()
        {
            mono_runtime_invoke(JacoArmUpdate_, arm_, NULL, NULL);
            MonoObject* mstate =
                mono_runtime_invoke(JacoArmState_, arm_, NULL, NULL);
            state_ = *((JacoArmState*)mono_object_unbox(mstate));
        }
	
	void retract()
	{
	    if (JacoRetract_)
	    {
            mono_runtime_invoke(JacoRetract_, arm_, NULL, NULL);
	    }
	    else
	    {
		    throw JacoArmException("Cannot find Retract method!");
	    }
	}
	

        void sendRelCartCommand(double x, double y, double z)
        {
            void* args[3];
            args[0] = &x;
            args[1] = &y;
            args[2] = &z;
            mono_runtime_invoke(JacoArmSendRelCartCommand_, arm_, args, NULL);
        }

        void sendAbsJointCommand(double joints[6])
        {
            void* args[6];
            for (unsigned int i = 0; i < 6; i++)
                args[i] = &joints[i];
            mono_runtime_invoke(JacoJointsAngleRads_, arm_, args, NULL);

        }
        
        void sendAbsFingerCommand(double fingers[3])
        {
	        void* args[3];
	        for (unsigned int i = 0; i < 3; i++)
	        	args[i] = &fingers[i];
			mono_runtime_invoke(JacoFingersAngleRads_, arm_, args, NULL);	        		      	      
        }
        

        void goToPose(double x, double y, double z, 
            double e_x, double e_y, double e_z)
        {
            double cmd[6] = {x, y, z, e_x, e_y, e_z};
            void* args[1] = {&cmd};
            mono_runtime_invoke(JacoArmGoToPose_, arm_, args, NULL);
        }

    private:
        bool init_;

        MonoDomain* domain_;
        MonoAssembly* assembly_;
        MonoImage* image_;
        MonoClass* JacoArm_;
        MonoMethod* JacoArmCtor_;
        MonoMethod* JacoArmIsEnabled_;
        MonoMethod* JacoArmState_;
        MonoMethod* JacoArmUpdate_;
        MonoMethod* JacoArmSendRelCartCommand_;
        MonoMethod* JacoArmGoToPose_;        
	    MonoMethod* JacoRetract_;
	    MonoMethod* JacoJointsAngleRads_;
	    MonoMethod* JacoFingersAngleRads_;
        MonoObject* arm_;
        guint32 arm_handle_;

        JacoArmState state_;

    };
}

using namespace jaco_api;

JacoArm::JacoArm(const char* key) throw(JacoArmException)
{
    impl_ = new JacoArmImpl(key);
}

JacoArm::~JacoArm()
{
    delete impl_;
}

bool JacoArm::isEnabled() const
{
    return impl_->isEnabled();
}

const JacoArmState& JacoArm::state() const
{
    return impl_->state();
}

void JacoArm::update()
{
    return impl_->update();
}

void JacoArm::sendRelCartCommand(double x, double y, double z)
{
    impl_->sendRelCartCommand(x, y, z);
}

void JacoArm::sendAbsJointCommand(double joints[6])
{
    impl_->sendAbsJointCommand(joints);
}

void JacoArm::sendAbsFingerCommand(double fingers[3])
{
	impl_->sendAbsFingerCommand(fingers);
}


void JacoArm::goToPose(double x, double y, double z,
    double e_x, double e_y, double e_z)
{
    impl_->goToPose(x, y, z, e_x, e_y, e_z);
}

void JacoArm::retract()
{
    impl_->retract();
}

