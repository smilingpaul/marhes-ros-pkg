#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include <stdlib.h>
#include <stdio.h>

void VRPN_CALLBACK handleTracker(void *userdata, const vrpn_TRACKERCB t)
{
  //this function gets called when the tracker's POSITION xform is updated

  //you can change what this callback function is called for by changing
  //the type of t in the function prototype above.
  //Options are:
  //   vrpn_TRACKERCB              position
  //   vrpn_TRACKERVELCB           velocity
  //   vrpn_TRACKERACCCB           acceleration
  //   vrpn_TRACKERTRACKER2ROOMCB  tracker2room transform 
  //                                 (comes from local or remote
  //                                  vrpn_Tracker.cfg file)
  //   vrpn_TRACKERUNIT2SENSORCB   unit2sensor transform (see above comment)
  //   vrpn_TRACKERWORKSPACECB     workspace bounding box (extent of tracker)
  // userdata is whatever you passed into the register_change_handler function.
  // vrpn sucks it up and spits it back out at you. It's not used by vrpn internally
  ros::Time time_now;
  static tf::TransformBroadcaster tf_broadcaster;
  time_now = ros::Time::now();
  ROS_INFO("handle_tracker\tSensor %d is now at (%g,%g,%g)\n", t.sensor, t.pos[0], t.pos[1], t.pos[2]);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(t.pos[0] / 1000, t.pos[1] / 1000, t.pos[2] / 1000));
  transform.setRotation(tf::Quaternion(t.quat[0], t.quat[1], t.quat[2], t.quat[3]));
  tf::StampedTransform stampTransform(transform, time_now, "/vicon_world", "Pioneer01");
  tf_broadcaster.sendTransform(stampTransform);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_vrpn_client_simple");
	ros::NodeHandle n;

  vrpn_Tracker_Remote *tkr;
  tkr = new vrpn_Tracker_Remote("Pioneer01@Vicon");
  tkr->register_change_handler(NULL, handleTracker);

  while(ros::ok())
  {
    tkr->mainloop();
  }
}
