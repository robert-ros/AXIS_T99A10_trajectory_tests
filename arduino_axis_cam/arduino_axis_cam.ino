#include <ros.h>
#include <Servo.h> 
#include <axis_camera/Axis.h>


using axis_camera::Axis;
axis_camera::Axis axis_current_position;

ros::NodeHandle  nh;


Servo pan_motor;

float pan_setpoint = 0.0;
float tilt_setpoint = 0.0;


void axis_cmd_callback(const axis_camera::Axis& msg){

  pan_setpoint = msg.pan;
  tilt_setpoint = msg.tilt;

  pan_motor.write(pan_setpoint);
  
  
}


ros::Publisher axis_state("state", &axis_current_position);
ros::Subscriber<axis_camera::Axis> axis_cmd("cmd", axis_cmd_callback);


void setup() {


  nh.initNode();
  nh.subscribe(axis_cmd);
  nh.advertise(axis_state);

  pan_motor.attach(9); 


}

void loop() {


  axis_current_position.pan = float(pan_motor.read());
  
  axis_state.publish(&axis_current_position);
  
  nh.spinOnce();
  delay(100);

}
