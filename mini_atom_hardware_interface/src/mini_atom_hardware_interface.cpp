#include <sstream>
#include <mini_atom_hardware_interface/mini_atom_hardware_interface.h>


using namespace hardware_interface;


namespace mini_atom_hardware_interface
{
    MiniAtomHardwareInterface::MiniAtomHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &MiniAtomHardwareInterface::update, this);
    
        hardware_sub = nh_.subscribe("state", 1, &MiniAtomHardwareInterface::hardwareCallback, this);
        hardware_pub = nh_.advertise<axis_camera::Axis>("cmd", 1000);
    
    }

    MiniAtomHardwareInterface::~MiniAtomHardwareInterface() {

    }

    void MiniAtomHardwareInterface::init() {
        // Get joint names
        nh_.getParam("hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {


             // Create joint state interface: used for read()
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
             joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface: used for write()
            // JointHandle jointPositiontHandle(jointStateHandle, &joint_position_command_[i]);
            // position_joint_interface_.registerHandle(jointPositiontHandle);

            // Create velocity joint interface: used for write()
            JointHandle jointVelocitytHandle(jointStateHandle, &joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(jointVelocitytHandle);
        }

        registerInterface(&joint_state_interface_);
        //registerInterface(&position_joint_interface_);
        registerInterface(&velocity_joint_interface_);

    }

    void MiniAtomHardwareInterface::hardwareCallback(const axis_camera::Axis::ConstPtr& msg)
   
    {
           pan_motor = (msg->pan)*(3.14159265359/180.0) ; //deg to rad
           tilt_motor = (msg->tilt)*(3.14159265359/180.0 ); 
    }

    void MiniAtomHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write();
    }

    void MiniAtomHardwareInterface::read() {

        // joint_velocity_[0] = pan_motor;
        // joint_velocity_[1] = tilt_motor;

        joint_position_[0] = pan_motor; // pan state (updated in hardwareCallback)
        joint_position_[1] = tilt_motor; // tilt state (updated in hardwareCallback)



    }

    void MiniAtomHardwareInterface::write() {

        axis_camera::Axis axis_setpoint;

        // rad to deg
        axis_setpoint.pan = (180.0/3.14159265359)* joint_position_command_[0];  // pan setpoint
        axis_setpoint.tilt = (180.0/3.14159265359)* joint_position_command_[1]; // tilt setpoint

        hardware_pub.publish(axis_setpoint); 


        // pan_motor = joint_position_command_[0];
        // tilt_motor = joint_position_command_[1];


    }
}