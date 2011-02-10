#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_subject.h"
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

      if (!n_private.hasParam("subject_list"))
        ROS_FATAL("Error: subject_list was not provided");

      XmlRpc::XmlRpcValue subject_list;
      n_private.getParam("subject_list", subject_list);
      ROS_ASSERT(subject_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

      num_subjects = subject_list.size();

      if (num_subjects > MAX_SUBJECTS)
        ROS_FATAL("Tracking limit is %d subjects.", MAX_SUBJECTS);

      for (int32_t i = 0; i < num_subjects; ++i) 
      {
        ROS_ASSERT(subject_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        subjects[i] = (string)subject_list[i];       
        addresses[i] = subjects[i] + "@" + host_addr;
        vrpn_subjects[i] = new VRPNSubject(subjects[i], addresses[i], tf_ref_frame, tf_broadcaster);
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
    VRPNSubject * vrpn_subjects[MAX_SUBJECTS]; 
    tf::TransformBroadcaster tf_broadcaster;
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
  VRPNHandler my_handler(n);

  while (ros::ok()) 
  {
		my_handler.loop();
  }
}
