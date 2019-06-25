#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <vector>
#include <memory>

class HeaderChecker {
public:
    HeaderChecker(std::string name, double expected_hz)
        :name_(name)
        ,expected_interval_(1000. / expected_hz)
        ,count_(0)
        ,max_interval_(std::numeric_limits<double>::min())
        ,min_interval_(std::numeric_limits<double>::max())
        ,mean_interval_(0.)
        ,abs_error_(0.)
        ,negative_(0)
        ,large_(0)
    {}

    ~HeaderChecker()
    {
        if (count_ == 0) return;
        mean_interval_ /= count_;
        abs_error_ /= count_;
        printf("%s: %u messages; average interval %lf ms (max %lf, min %lf; %u negative, %u large)\n\t\texpected interval %lf ms, mean error %lf ms\n",
            name_.c_str(), count_, mean_interval_, max_interval_, min_interval_, negative_, large_, expected_interval_, abs_error_);
    }

    void check(const std_msgs::Header& header)
    {
        while (count_ != 0) {
            if (header.seq < last_seq_ + 1) {
                ROS_ERROR("%s: message comes out of order! (seq %u -> %u)", name_.c_str(), last_seq_, header.seq);
                break;
            } else if (header.seq > last_seq_ + 1) {
                ROS_ERROR("%s: message drop! (seq %u -> %u)", name_.c_str(), last_seq_, header.seq);
            }
            double interval = header.stamp.toSec() * 1000. - last_stamp_;
            if (interval > max_interval_) max_interval_ = interval;
            if (interval < min_interval_) min_interval_ = interval;
            mean_interval_ += interval;
            abs_error_ += std::abs(interval - expected_interval_);
            if (interval > 1.5 * expected_interval_) {
                ROS_WARN("%s: seq %u->%u: large interval %lf ms (expect %lf), may be frame drop", name_.c_str(), last_seq_, header.seq, interval, expected_interval_);
                ++large_;
            } else if (interval < 0) {
                ROS_WARN("%s: seq %u->%u: negative interval %lf ms", name_.c_str(), last_seq_, header.seq, interval);
                ++negative_;
            }
            break;
        }
        last_seq_ = header.seq;
        last_stamp_ = header.stamp.toSec() * 1000.;
        ++count_;
    }

    template <typename T>
    void callback(const T& msg)
    {
        check(msg.header);
    }

    const std::string name_;
    const double expected_interval_;
    uint32_t count_;
    uint32_t last_seq_;
    double last_stamp_;
    double max_interval_, min_interval_, mean_interval_;
    double abs_error_;
    uint32_t negative_, large_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "check_sequence");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string config_file;
    pnh.param("config", config_file, std::string());

    std::ifstream file(config_file.c_str());
    if(!file.is_open()) {
        ROS_ERROR("Cannot open the specified config file \"%s\"", config_file.c_str());
        return 0;
    }

    std::vector<std::unique_ptr<HeaderChecker>> checkers;
    std::vector<ros::Subscriber> subs;
    for (std::string line; std::getline(file, line);) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream sline(line);
        std::string name, type, topic;
        double fps;
        sline >> name >> type >> fps >> topic;
        checkers.emplace_back(std::make_unique<HeaderChecker>(name, fps));
        bool subscribed = true;
        if (type == "Image") subs.emplace_back(nh.subscribe(topic, 2000, &HeaderChecker::callback<sensor_msgs::Image>, checkers.back().get()));
        else if (type == "Imu") subs.emplace_back(nh.subscribe(topic, 2000, &HeaderChecker::callback<sensor_msgs::Imu>, checkers.back().get()));
        else {
            subscribed = false;
            ROS_WARN("Unsupported message type \"%s\"", type.c_str());
        }
        if (subscribed) {
            ROS_INFO("Subscribed to %s (expected FPS: %.1lf)", topic.c_str(), fps);
        } else {
            checkers.pop_back();
        }
    }
    file.close();

    ROS_INFO("start check...");
    ros::spin();
    printf("\n");

    return 0;
}
