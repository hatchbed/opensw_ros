/**
 * Copyright (c) 2022, Hatchbed
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <opensw/client.h>
#include <opensw_ros/logger.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opensw_imu_driver");
    ros::NodeHandle node;
    ros::NodeHandle priv("~");

    opensw_ros::LogBridge log_bridge("opensw", node);

    ROS_INFO("Initializing opensw imu driver ...");

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

    opensw::Client client;

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

        std::optional<opensw::ImuData> imu_data;
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
