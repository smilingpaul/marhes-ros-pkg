#include "ros/ros.h"
#include "vicon_multicast/Client.h"

#define NOT_CONNECTED       0
#define SETUP_MULTICAST     1
#define GET_FRAMES          2
#define LIMIT_FAIL_CONNECT  5

using namespace ViconDataStreamSDK::CPP;

int main(int argc, char **argv)
{   
    // ROS initialization
	ros::init(argc, argv, "vicon_multicast_handler");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

    // Variable declaration
	Client multicast_handler;
    std::string host_addr, multicast_addr;
    int state = NOT_CONNECTED;
    int num_fail_connect = 0;
	ros::Rate loop_rate(10);

	n_private.param("host_addr", host_addr, std::string("Vicon:800"));
	n_private.param("multicast_addr", multicast_addr, std::string("224.0.0.0"));

	if (multicast_handler.IsConnected().Connected)
		state = SETUP_MULTICAST;

	while(ros::ok())
	{
		switch(state)
		{
			case NOT_CONNECTED:
			{
				Output_Connect out_conn = multicast_handler.Connect(host_addr);

				switch(out_conn.Result)
				{
					case Result::Success:
					{
						state = SETUP_MULTICAST;
						num_fail_connect = 0;
						ROS_INFO("Connected to Vicon Server.");
					}
					break;
					case Result::InvalidHostName:
					{
						ROS_FATAL("Failed to Connect: Invalid Host Name.");
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
//				multicast_handler.EnableDeviceData();
//				multicast_handler.EnableMarkerData();
				multicast_handler.EnableSegmentData();
//				multicast_handler.EnableUnlabeledMarkerData();

				Output_StartTransmittingMulticast out_mult = multicast_handler.
					StartTransmittingMulticast(host_addr, multicast_addr);

				switch(out_mult.Result)
				{
					case Result::Success:
					{
						state = GET_FRAMES;
						ROS_INFO("Server started transmitting multicast.");
					}
					break;
					case Result::NotConnected:
					{
						state = NOT_CONNECTED;
						ROS_WARN("Attempted to start multicast, but no connection."
								"Trying to reconnect.");
					}
					break;
					case Result::InvalidMulticastIP:
					{
						ROS_FATAL("Failed to Start Multicasting: Invalid Multicast IP.");
					}
					break;
					case Result::ServerAlreadyTransmittingMulticast:
					{
						state = GET_FRAMES;
						ROS_WARN("Server Already Transmitting Multicast.");

					}
					break;
					default:
						break;
				}
			}
			break;

			case GET_FRAMES:
				if (!multicast_handler.IsConnected().Connected)
				{
					state = NOT_CONNECTED;
					multicast_handler.StopTransmittingMulticast();
					multicast_handler.Disconnect();
				}
				break;
			default:
				break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	multicast_handler.StopTransmittingMulticast();
	multicast_handler.Disconnect();

  	return 0;
}
