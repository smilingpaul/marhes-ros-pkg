#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_Tracker.h"
//#include "vicon_vrpn/vrpn_subject.h"
#include <stdlib.h>
#include <stdio.h>

#define MAX_SUBJECTS  100

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

      if (!n.hasParam("subject_list"))
        ROS_FATAL("Error: subject_list was not provided");

      XmlRpc::XmlRpcValue subject_list;
      n.getParam("subject_list", subject_list);
      ROS_ASSERT(subject_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

      num_subjects = subject_list.size();

      if (num_subjects > MAX_SUBJECTS)
        ROS_FATAL("Tracking limit is %d subjects.", MAX_SUBJECTS);

      for (int32_t i = 0; i < num_subjects; ++i) 
      {
        ROS_ASSERT(subject_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        subjects[i] = (string)subject_list[i];       
        addresses[i] = subjects[i] + "@" + host_addr;
        
        // Open the tracker
        tkr[i] = new vrpn_Tracker_Remote(addresses[i].c_str());
        // Set up the tracker callback handler
        tkr[i]->register_change_handler(NULL, (vrpn_TRACKERCHANGEHANDLER)&handleTracker);

        //vrpn_subjects[i] = new VRPNSubject(subjects[i], addresses[i], tf_ref_frame, tf_broadcaster);
        ROS_INFO("Tracking subject at: %s", ((string)addresses[i]).c_str());
      } 
    }

    void loop(void)
    {
      for (int i = 0; i < num_subjects; i++)
        vrpn_subjects[i]->loop();
    }

  private:
    ros::NodeHandle n;
    string host_addr, tf_ref_frame, subjects[MAX_SUBJECTS], addresses[MAX_SUBJECTS];
    int num_subjects;
    //VRPNSubject * vrpn_subjects[MAX_SUBJECTS]; 
    vrpn_Tracker_Remote tkr[MAX_SUBJECTS];
    tf::TransformBroadcaster tf_broadcaster;

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
      time_now = ros::Time::now();
      //ROS_INFO("handle_tracker\tSensor %d is now at (%g,%g,%g)\n", t.sensor, t.pos[0], t.pos[1], t.pos[2]);
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(t.pos[0] / 1000, t.pos[1] / 1000, t.pos[2] / 1000));
      transform.setRotation(tf::Quaternion(t.quat[0], t.quat[1], t.quat[2], t.quat[3]));
      tf::StampedTransform stampTransform(transform, time_now, tf_ref_frame, subject);
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
  VRPNHandler *my_handler = new VRPNHandler(n);

  while (ros::ok()) 
  {
    my_handler->loop();
  }
}
