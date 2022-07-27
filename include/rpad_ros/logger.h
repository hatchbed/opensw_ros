#include <string>

#include <ros/ros.h>

namespace rpad_ros {

class LogBridge {

    public:
    LogBridge(const std::string& name, const ros::NodeHandle& node);

    private:
    void checkLogLevel(const ros::WallTimerEvent& e);

    std::string name_;
    std::string full_name_;
    ros::NodeHandle node_;
    ros::WallTimer timer_;
    ros::console::levels::Level log_level_ = ros::console::levels::Info;
};

}  // namespace rpad_ros
