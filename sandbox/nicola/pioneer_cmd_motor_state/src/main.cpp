#include <ros/ros.h>
#include <p2os_driver/MotorState.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "pioneer_motor_state_publisher");
  ros::NodeHandle n;

  ros::Rate r(1);

  ros::Publisher motor_state_pub = n.advertise<p2os_driver::MotorState>("/cmd_motor_state", 1);

  while(n.ok()){
    p2os_driver::MotorState state_msg;
    state_msg.state = 1;
    motor_state_pub.publish(state_msg);	  

    ros::spinOnce();
    r.sleep();
  }
}

