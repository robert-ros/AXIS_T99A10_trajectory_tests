#ifndef ROS_CONTROL__MINI_ATOM_HARDWARE_INTERFACE_H
#define ROS_CONTROL__MINI_ATOM_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <mini_atom_hardware_interface/mini_atom_hardware.h>
#include "axis_camera/Axis.h"

using namespace hardware_interface;


namespace mini_atom_hardware_interface
{
    static const double POSITION_STEP_FACTOR = 10;
    static const double VELOCITY_STEP_FACTOR = 10;

    class MiniAtomHardwareInterface: public mini_atom_hardware_interface::MiniAtomHardware
    {
        public:
            MiniAtomHardwareInterface(ros::NodeHandle& nh);
            ~MiniAtomHardwareInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read();
            void write();


        protected:
            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration control_period_;
            ros::Duration elapsed_time_;
            PositionJointInterface positionJointInterface;
            VelocityJointInterface velocityJointInterface;
            EffortJointInterface effortJointInterface;
            double loop_hz_;
            boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
            double p_error_, v_error_, e_error_;
        
        private:
            float pan_motor;
            float tilt_motor;
            ros::Subscriber hardware_sub;
            ros::Publisher hardware_pub;
            void hardwareCallback(const axis_camera::Axis::ConstPtr& msg);


    };

}

#endif

