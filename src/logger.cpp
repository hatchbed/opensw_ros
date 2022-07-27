#include <rpad_ros/logger.h>

#include <map>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <rpad/logger.h>

namespace spd = spdlog;

namespace rpad_ros {

template<typename Mutex>
class RosLogSink : public spdlog::sinks::base_sink <Mutex>
{
    public:
    RosLogSink(const std::string& name) : name_(name) {}

    protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        if (msg.level == spdlog::level::info) {
            ROS_INFO_STREAM_NAMED(name_, fmt::to_string(msg.payload));
        }
        else if (msg.level == spdlog::level::warn) {
            ROS_WARN_STREAM_NAMED(name_, fmt::to_string(msg.payload));
        }
        else if (msg.level == spdlog::level::err) {
            ROS_ERROR_STREAM_NAMED(name_, fmt::to_string(msg.payload));
        }
        else {
            ROS_DEBUG_STREAM_NAMED(name_, fmt::to_string(msg.payload));
        }
    }

    void flush_() override {}

    std::string name_;
};

LogBridge::LogBridge(const std::string& name, const ros::NodeHandle& node) :
    name_(name),
    full_name_("ros.rpad_ros." + name_),
    node_(node)
{
    spdlog::sink_ptr ros_sink = std::make_shared<RosLogSink<std::mutex>>(name_);

    spdlog::default_logger()->sinks().clear();
    spdlog::default_logger()->sinks().push_back(ros_sink);
    spdlog::set_pattern("%v");
    ROS_INFO_STREAM_NAMED(name_, "Initialized log bridge");
    ros::console::set_logger_level(full_name_, ros::console::levels::Info);

    timer_ = node_.createWallTimer(ros::WallDuration(1.0), &LogBridge::checkLogLevel, this, false);
}

void LogBridge::checkLogLevel(const ros::WallTimerEvent& e) {
    std::map<std::string, ros::console::levels::Level> loggers;
    ros::console::get_loggers(loggers);
    auto level = loggers[full_name_];
    if (level != log_level_) {
        log_level_ = level;

        if (level == ros::console::levels::Info) {
            spdlog::set_level(spdlog::level::info);
        }
        else if (level == ros::console::levels::Debug) {
            spdlog::set_level(spdlog::level::debug);
        }
        else if (level == ros::console::levels::Warn) {
            spdlog::set_level(spdlog::level::warn);
        }
        else if (level == ros::console::levels::Error) {
            spdlog::set_level(spdlog::level::err);
        }
        else if (level == ros::console::levels::Fatal) {
            spdlog::set_level(spdlog::level::err);
        }
    }
}

}  // namespace rpad_ros
