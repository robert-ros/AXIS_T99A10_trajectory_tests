#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <mini_atom_hardware_interface/mini_atom_hardware_interface.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mini_atom_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    mini_atom_hardware_interface::MiniAtomHardwareInterface rhi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
   
    return 0;
}