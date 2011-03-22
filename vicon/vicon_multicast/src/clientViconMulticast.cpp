#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_multicast/Client.h"

#define NOT_CONNECTED       0
#define SETUP_MULTICAST     1
#define GET_FRAMES          2
#define LIMIT_FAIL_CONNECT  5

using namespace ViconDataStreamSDK::CPP;

Direction::Enum Adapt(std::string direction)
{
    if (direction == "Forward")
        return Direction::Forward;
    else if (direction == "Backward")
        return Direction::Backward;
    else if (direction == "Left")
        return Direction::Left;
    else if (direction == "Right")
        return Direction::Right;
    else if (direction == "Up")
        return Direction::Up;
    else if (direction == "Down")
        return Direction::Down;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_multicast_client");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	Client my_client;
	int state = NOT_CONNECTED;
	int num_fail_connect = 0;
	ros::Rate loop_rate(100);
	ros::Time time_now;
	std::string host_addr, multicast_addr, stream_mode, tf_ref_frame;
	std::string x_axis_mapping, y_axis_mapping, z_axis_mapping;
	Result::Enum output;
	tf::TransformBroadcaster tf_broadcaster;

	n_private.param("host_addr", host_addr, std::string("Vicon:800"));
	n_private.param("multicast_addr", multicast_addr, std::string("224.0.0.0"));
	n_private.param("stream_mode", stream_mode, std::string("ClientPullPreFetch"));
	n_private.param("x_axis", x_axis_mapping, std::string("Forward"));
	n_private.param("y_axis", y_axis_mapping, std::string("Left"));
	n_private.param("z_axis", z_axis_mapping, std::string("Up"));
	n_private.param("tf_ref_frame", tf_ref_frame, std::string("/vicon_world"));
  	while (ros::ok())
  	{
  		switch(state)
  		{
			case NOT_CONNECTED:
			{
				output = my_client.ConnectToMulticast(host_addr, multicast_addr).Result;

				switch(output)
				{
					case Result::Success:
					{
						state = SETUP_MULTICAST;
						num_fail_connect = 0;
						ROS_INFO("Connected to multicast Vicon server.");
					}
					break;
					case Result::InvalidHostName:
					{
						ROS_FATAL("Failed to Connect: Invalid Host Name.");
					}
					break;
					case Result::InvalidMulticastIP:
					{
						ROS_FATAL("Failed to Start Multicasting: Invalid Multicast IP.");
					}
					break;
					case Result::ClientAlreadyConnected:
					{
						state = SETUP_MULTICAST;
						num_fail_connect = 0;
						ROS_WARN("Already connected to Vicon Server.");
					}
					break;
					case Result::ClientConnectionFailed:
					{
						num_fail_connect++;
						ROS_WARN("Failed to Connect.  Attempt: %i.", num_fail_connect);

						if (num_fail_connect > LIMIT_FAIL_CONNECT)
							ROS_FATAL("Failed to Connect: Attempts exceeded the limit.");
					}
					break;
					default:
						break;
				}
			}
			break;

			case SETUP_MULTICAST:
			{
                if (stream_mode == "ServerPush")
			    	output = my_client.SetStreamMode(StreamMode::ServerPush).Result;
			    else if (stream_mode == "ClientPull")
			    	output = my_client.SetStreamMode(StreamMode::ClientPull).Result;
			    else if (stream_mode == "ClientPullPreFetch")
			    	output = my_client.SetStreamMode(StreamMode::ClientPullPreFetch).Result;
			    else
			    	ROS_FATAL("Failed to set stream mode: Unknown Vicon Stream Mode.");

			    if (output == Result::Success)
			    {
			    	output = my_client.SetAxisMapping(Adapt(x_axis_mapping),
			    			Adapt(y_axis_mapping), Adapt(z_axis_mapping)).Result;
			    	if (output == Result::Success)
			    		state = GET_FRAMES;
			    	else
			    		ROS_FATAL("Failed to set axis mapping.");
			    }
			    else 	// Not connected
			    {
			    	state = NOT_CONNECTED;
			    }
			}
			break;

			case GET_FRAMES:
			{
				if (my_client.GetFrame().Result == Result::Success)
				{
					time_now = ros::Time::now();
					Output_GetSubjectCount output_count = my_client.GetSubjectCount();
					if (output_count.Result == Result::Success)
					{
						for(unsigned int i = 0; i < output_count.SubjectCount; i++)
						{
							std::string name = my_client.GetSubjectName(i).SubjectName;
							std::string segment = my_client.GetSubjectRootSegmentName(name).SegmentName;
							Output_GetSegmentGlobalTranslation out_trans =
									my_client.GetSegmentGlobalTranslation(name, segment);
							Output_GetSegmentGlobalRotationQuaternion out_rot =
									my_client.GetSegmentGlobalRotationQuaternion(name, segment);

							if (out_trans.Occluded && out_rot.Occluded)
							{
								tf::Transform transform;
								transform.setOrigin(tf::Vector3(out_trans.Translation[0] / 1000,
										out_trans.Translation[1] / 1000, out_trans.Translation[2] / 1000));
								transform.setRotation(tf::Quaternion(out_rot.Rotation[0] / 1000,
										out_rot.Rotation[1] / 1000, out_rot.Rotation[2] / 1000,
										out_rot.Rotation[3] / 1000));
								tf::StampedTransform stampTransform(transform, time_now, tf_ref_frame, name);
								tf_broadcaster.sendTransform(stampTransform);
							}
						}
					}
				}
			}
			break;

			default:
				break;

  		}

      	ros::spinOnce();
    	loop_rate.sleep();
  	}

  	my_client.Disconnect();

  	return 0;
}
