#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include <stdlib.h>
#include <stdio.h>

using namespace std;

class VRPNHandler
{
  public:
    VRPNHandler(ros::NodeHandle nh)
    {
      n = nh;
      ros::NodeHandle n_private("~"); 
      n_private.param("host_addr", host_addr, string("Vicon"));
    	n_private.param("tf_ref_frame", tf_ref_frame, string("/vicon_world"));     
      n_private.param("subjects", subjects, string("Pioneer01"));
      
      address = subjects + "@" + host_addr;

      // Open the tracker
      tkr = new vrpn_Tracker_Remote(address.c_str());
	    // Set up the tracker callback handler
      tkr->register_change_handler(NULL, (vrpn_TRACKERCHANGEHANDLER)&VRPNHandler::handle_tracker);
    }

    void Loop(void)
    {
      tkr->mainloop();
    }
  private:
    ros::NodeHandle n;
    string host_addr, tf_ref_frame, subjects, address;
    vrpn_Tracker_Remote *tkr; 
    tf::TransformBroadcaster tf_broadcaster;

    void VRPN_CALLBACK handle_tracker(void *userdata, const vrpn_TRACKERCB t)
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
      ROS_INFO("handle_tracker\tSensor %d is now at (%g,%g,%g)\n", t.sensor, t.pos[0], t.pos[1], t.pos[2]);
      time_now = ros::Time::now();
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(t.pos[0] / 1000, t.pos[1] / 1000, t.pos[2] / 1000));
      transform.setRotation(tf::Quaternion(t.quat[0], t.quat[1], t.quat[2], t.quat[3]));
      tf::StampedTransform stampTransform(transform, time_now, "/vicon_world", "Pioneer01");
      tf_broadcaster.sendTransform(stampTransform);
    }
};

//****************************************************************************
//
//   Main function
//
//****************************************************************************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_vrpn_client");
	ros::NodeHandle n;
  VRPNHandler myHandler(n);

  while (ros::ok()) 
  {
		myHandler.Loop();
  }
}
