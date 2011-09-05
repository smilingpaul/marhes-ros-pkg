#ifndef VRPN_SUBJECT_
#define VRPN_SUBJECT_

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include "vicon_vrpn/vrpn_subject.h"
#include <stdlib.h>
#include <stdio.h>

using namespace std;

class VRPNSubject
{
public:
  VRPNSubject(string subj, string addr, string global_frame, tf::TransformBroadcaster tf_b);
  void loop(void);
private:
  string tf_ref_frame, subject, address; 
  vrpn_Tracker_Remote *tkr;
  tf::TransformBroadcaster tf_broadcaster;
  
  void VRPN_CALLBACK handleTracker(void *userdata, const vrpn_TRACKERCB t);
};
#endif
