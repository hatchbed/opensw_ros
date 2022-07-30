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

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rpad/client.h>
#include <rpad_ros/logger.h>
#include <rpad_ros/params.h>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using namespace rpad_ros;

int main(int argc, char **argv)
{
    auto node = std::make_shared<rclcpp::Node>("rpad_laser_driver");

    LogBridge log_bridge("rpad", node);

    RCLCPP_INFO(node->get_logger(), "Initializing rpad laser driver ...");

    std::string host = param(node, "host", std::string("192.168.11.11"), "Host to connect to");
    RCLCPP_INFO(node->get_logger(),"  host: %s", host.c_str());

    int port = param(node, "port", 1445, "TCP port to connect to");
    RCLCPP_INFO(node->get_logger(),"  port: %d", port);

    std::string frame_id = param(node, "frame_id", std::string("laser"), "Frame to use for the published messages");
    RCLCPP_INFO(node->get_logger(),"  frame_id: %s", frame_id.c_str());

    float min_range = param(node, "min_range", 0.0, "Min sensor range in meters");
    RCLCPP_INFO(node->get_logger(),"  min_range: %f", min_range);

    float max_range = param(node, "max_range", 40.0, "Max sensor range in meters");
    RCLCPP_INFO(node->get_logger(),"  max_range: %f", max_range);

    float rate = param(node, "rate", 20.0, "Publish rate");
    RCLCPP_INFO(node->get_logger(),"  rate: %f", rate);

    auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

    rpad::Client client;

    rclcpp::Time last_stamp = node->now();
    rclcpp::Rate spin_rate(rate);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (!client.connected()) {
            RCLCPP_INFO(node->get_logger(),"Connecting to device ...");
            if (!client.connect(host, port, 500)) {
                RCLCPP_WARN(node->get_logger(),"Failed to connect to device.");
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(1.0s));
                continue;
            }
        }

        auto scan = client.getLaserScan();
        if (!scan || scan->points.size() < 2) {
            RCLCPP_WARN(node->get_logger(),"Failed to get laser scan.  Disconnecting...");
            client.disconnect();
            continue;
        }

        rclcpp::Time stamp = node->now();
        double scan_time = (stamp - last_stamp).seconds();
        last_stamp = stamp;

        RCLCPP_DEBUG(node->get_logger(), "got scan with %zu points", scan->points.size());
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.angle_min = scan->points.front().angle;
        msg.angle_max = scan->points.back().angle;
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (scan->points.size() - 1);
        msg.scan_time = scan_time;
        msg.time_increment = scan_time / (scan->points.size() - 1);
        msg.range_min = min_range;
        msg.range_max = max_range;
        msg.ranges.resize(scan->points.size());
        for (size_t i = 0; i < scan->points.size(); i++) {
            msg.ranges[i] = scan->points[i].distance;
        }
        scan_pub->publish(msg);
        spin_rate.sleep();
    }

    return 0;
}
