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

#include <rpad_ros/logger.h>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <rpad/logger.h>

using namespace std::chrono_literals;

namespace spd = spdlog;

namespace rpad_ros {

template<typename Mutex>
class RosLogSink : public spdlog::sinks::base_sink <Mutex>
{
    public:
    RosLogSink(const rclcpp::Logger& logger) : logger_(logger) {}

    protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        static rcutils_log_location_t __log_loc = { msg.source.funcname, msg.source.filename, static_cast<size_t>(msg.source.line) };
        if (msg.level == spdlog::level::info) {
            rcutils_log(&__log_loc, RCUTILS_LOG_SEVERITY_INFO, logger_.get_name(), "%s", fmt::to_string(msg.payload).c_str());
        }
        else if (msg.level == spdlog::level::warn) {
            rcutils_log(&__log_loc, RCUTILS_LOG_SEVERITY_WARN, logger_.get_name(), "%s", fmt::to_string(msg.payload).c_str());
        }
        else if (msg.level == spdlog::level::err) {
            rcutils_log(&__log_loc, RCUTILS_LOG_SEVERITY_ERROR, logger_.get_name(), "%s", fmt::to_string(msg.payload).c_str());
        }
        else {
            rcutils_log(&__log_loc, RCUTILS_LOG_SEVERITY_DEBUG, logger_.get_name(), "%s", fmt::to_string(msg.payload).c_str());
        }
    }

    void flush_() override {}

    rclcpp::Logger logger_;
};

LogBridge::LogBridge(const std::string& name, rclcpp::Node::SharedPtr node) :
    node_(node)
{
    auto logger = node->get_logger();//.get_child(name);
    full_name_ = logger.get_name();

    spdlog::sink_ptr ros_sink = std::make_shared<RosLogSink<std::mutex>>(logger);

    spdlog::default_logger()->sinks().clear();
    spdlog::default_logger()->sinks().push_back(ros_sink);
    spdlog::set_pattern("%v");

    RCLCPP_INFO_STREAM(logger, "Initialized log bridge");

    timer_ = node_->create_wall_timer(1000ms, [&, logger]()
    {
        auto level = rcutils_logging_get_logger_effective_level(full_name_.c_str());
        if (level != log_level_) {
            if (level == RCUTILS_LOG_SEVERITY_INFO) {
                log_level_ = RCUTILS_LOG_SEVERITY_INFO;
                spdlog::set_level(spdlog::level::info);
            }
            else if (level == RCUTILS_LOG_SEVERITY_DEBUG) {
                log_level_ = RCUTILS_LOG_SEVERITY_DEBUG;
                spdlog::set_level(spdlog::level::debug);
            }
            else if (level == RCUTILS_LOG_SEVERITY_WARN) {
                log_level_ = RCUTILS_LOG_SEVERITY_WARN;
                spdlog::set_level(spdlog::level::warn);
            }
            else if (level == RCUTILS_LOG_SEVERITY_ERROR) {
                log_level_ = RCUTILS_LOG_SEVERITY_ERROR;
                spdlog::set_level(spdlog::level::err);
            }
            else if (level == RCUTILS_LOG_SEVERITY_FATAL) {
                log_level_ = RCUTILS_LOG_SEVERITY_FATAL;
                spdlog::set_level(spdlog::level::err);
            }
        }
    });
}

}  // namespace rpad_ros
