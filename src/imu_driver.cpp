
#include <rpad/client.h>
#include <rpad_ros/logger.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpad_imu_driver");
    ros::NodeHandle node;
    ros::NodeHandle priv("~");

    rpad_ros::LogBridge log_bridge("rpad", node);

    ROS_INFO("Initializing rpad imu driver ...");

    std::string host = priv.param("host", std::string("192.168.11.11"));
    ROS_INFO("  host: %s", host.c_str());

    int port = priv.param("port", 1445);
    ROS_INFO("  port: %d", port);

    std::string frame_id = priv.param("frame_id", std::string("imu"));
    ROS_INFO("  frame_id: %s", frame_id.c_str());

    bool use_raw_data = priv.param("use_raw_data", true);
    ROS_INFO("  use_raw_data: %s", (use_raw_data ? "true" : "false"));

    float rate = priv.param("rate", 50.0);
    ROS_INFO("  rate: %f", rate);

    ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("imu", 10);

    rpad::Client client;

    ros::Time last_stamp = ros::Time::now();
    ros::Rate spin_rate(rate);
    while (ros::ok()) {
        ros::spinOnce();

        if (!client.connected()) {
            ROS_INFO("Connecting to device ...");
            if (!client.connect(host, port, 500)) {
                ROS_WARN("Failed to connect to device.");
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        std::optional<rpad::ImuData> imu_data;
        if (use_raw_data) {
            imu_data = client.getRawImuData();
        }
        else {
            imu_data = client.getImuData();
        }

        if (!imu_data) {
            ROS_WARN("Failed to get imu data.  Disconnecting...");
            client.disconnect();
            continue;
        }

        ros::Time stamp = ros::Time::now();

        auto msg = boost::make_shared<sensor_msgs::Imu>();
        msg->header.stamp = stamp;
        msg->header.frame_id = frame_id;

        msg->orientation.w = imu_data->orientation.w();
        msg->orientation.x = imu_data->orientation.x();
        msg->orientation.y = imu_data->orientation.y();
        msg->orientation.z = imu_data->orientation.z();

        msg->angular_velocity.x = imu_data->angular_rate.x();
        msg->angular_velocity.y = imu_data->angular_rate.y();
        msg->angular_velocity.z = imu_data->angular_rate.z();

        msg->linear_acceleration.x = imu_data->acceleration.x();
        msg->linear_acceleration.y = imu_data->acceleration.y();
        msg->linear_acceleration.z = imu_data->acceleration.z();

        imu_pub.publish(msg);
        spin_rate.sleep();
    }

    return 0;
}
