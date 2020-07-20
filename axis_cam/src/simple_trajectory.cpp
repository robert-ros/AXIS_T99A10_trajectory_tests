// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("axis_cam_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("joint_pan");
  goal.trajectory.joint_names.push_back("joint_tilt");


  // -----------  Number of waypoints in this goal trajectory ----------------- //
  goal.trajectory.points.resize(3);


  // -------------------------  First trajectory point -------------------------- //
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;

  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(1.0);


  // -------------------------  Second trajectory point -------------------------- //
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 1.57;
  goal.trajectory.points[index].positions[1] = 0.2;

  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);


  // -------------------------  Third trajectory point -------------------------- //

  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;

  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(7.0);

}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting run_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  waypoints_arm_goal(arm_goal);

  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }

  return EXIT_SUCCESS;
}



// #include <ros/ros.h>
// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <actionlib/client/simple_action_client.h>

// typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

// class RobotArm
// {
// private:
//   // Action client for the joint trajectory action 
//   // used to trigger the arm movement action
//   TrajClient* traj_client_;

// public:
//   //! Initialize the action client and wait for action server to come up
//   RobotArm() 
//   {
//     // tell the action client that we want to spin a thread by default
//     traj_client_ = new TrajClient("axis_cam_controller/follow_joint_trajectory", true);

//     // wait for action server to come up
//     while(!traj_client_->waitForServer(ros::Duration(5.0))){
//       ROS_INFO("Waiting for the joint_trajectory_action server");
//     }
//   }

//   //! Clean up the action client
//   ~RobotArm()
//   {
//     delete traj_client_;
//   }

//   //! Sends the command to start a given trajectory
//   void startTrajectory(control_msgs::JointTrajectoryGoal goal)
//   {
//     // When to start the trajectory: 1s from now
//     goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
//     traj_client_->sendGoal(goal);
//   }

//   //! Generates a simple trajectory with two waypoints, used as an example
//   /*! Note that this trajectory contains two waypoints, joined together
//       as a single trajectory. Alternatively, each of these waypoints could
//       be in its own trajectory - a trajectory can have one or more waypoints
//       depending on the desired application.
//   */
//   control_msgs::JointTrajectoryGoal armExtensionTrajectory()
//   {
//     //our goal variable
//     control_msgs::JointTrajectoryGoal goal;

//     // First, the joint names, which apply to all waypoints
//     goal.trajectory.joint_names.push_back("joint_pan");
//     goal.trajectory.joint_names.push_back("joint_tilt");


//     // We will have two waypoints in this goal trajectory
//     goal.trajectory.points.resize(2);

//     // First trajectory point
//     // Positions
//     int ind = 0;
//     goal.trajectory.points[ind].positions.resize(2);
//     goal.trajectory.points[ind].positions[0] = 0.0;
//     goal.trajectory.points[ind].positions[1] = 0.0;

//     // Velocities
//     goal.trajectory.points[ind].velocities.resize(2);
//     for (size_t j = 0; j < 2; ++j)
//     {
//       goal.trajectory.points[ind].velocities[j] = 0.0;
//     }
//     // To be reached 1 second after starting along the trajectory
//     goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

//     // Second trajectory point
//     // Positions
//     ind += 1;
//     goal.trajectory.points[ind].positions.resize(2);
//     goal.trajectory.points[ind].positions[0] = -0.3;
//     goal.trajectory.points[ind].positions[1] = 0.2;

//     // Velocities
//     goal.trajectory.points[ind].velocities.resize(2);
//     for (size_t j = 0; j < 7; ++j)
//     {
//       goal.trajectory.points[ind].velocities[j] = 0.0;
//     }
//     // To be reached 2 seconds after starting along the trajectory
//     goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

//     //we are done; return the goal
//     return goal;
//   }

//   //! Returns the current state of the action
//   actionlib::SimpleClientGoalState getState()
//   {
//     return traj_client_->getState();
//   }
 
// };

// int main(int argc, char** argv)
// {
//   // Init the ROS node
//   ros::init(argc, argv, "robot_driver");

//   RobotArm arm;
//   // Start the trajectory
//   arm.startTrajectory(arm.armExtensionTrajectory());
//   // Wait for trajectory completion
//   while(!arm.getState().isDone() && ros::ok())
//   {
//     usleep(50000);
//   }
//   return 0;
// }