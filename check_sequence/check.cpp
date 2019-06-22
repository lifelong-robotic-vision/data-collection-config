#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

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
        if (count_ != 0) {
            if (header.seq < last_seq_ + 1) {
                ROS_ERROR("%s: message comes out of order! (seq %u -> %u)", name_.c_str(), last_seq_, header.seq);
                return;
            } else if (header.seq > last_seq_ + 1) {
                ROS_ERROR("%s: message drop! (seq %u -> %u)", name_.c_str(), last_seq_, header.seq);
            }
            double interval = header.stamp.toSec() * 1000. - last_stamp_;
            if (interval > max_interval_) max_interval_ = interval;
            if (interval < min_interval_) min_interval_ = interval;
            mean_interval_ += interval;
            abs_error_ += std::abs(interval - expected_interval_);
            if (interval > 1.5 * expected_interval_) {
                ROS_WARN("%s: large interval %lf ms (expect %lf), may be frame drop", name_.c_str(), interval, expected_interval_);
                ++large_;
            } else if (interval < 0) {
                ROS_WARN("%s: negative interval %lf ms", name_.c_str(), interval);
                ++negative_;
            }
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
    ROS_INFO("start check...");
    ros::init(argc, argv, "check_msg");
    ros::NodeHandle nh;
    HeaderChecker color_checker("color", 30);
    HeaderChecker depth_checker("depth", 30);
    HeaderChecker gyro_checker("gyro", 400);
    HeaderChecker accel_checker("accel", 250);
    ros::Subscriber sub_color = nh.subscribe("/camera/color/image_raw", 2000, &HeaderChecker::callback<sensor_msgs::Image>, &color_checker);
    ros::Subscriber sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 2000, &HeaderChecker::callback<sensor_msgs::Image>, &depth_checker);
    ros::Subscriber sub_gyro = nh.subscribe("/camera/gyro/sample", 2000, &HeaderChecker::callback<sensor_msgs::Imu>, &gyro_checker);
    ros::Subscriber sub_accel = nh.subscribe("/camera/accel/sample", 2000, &HeaderChecker::callback<sensor_msgs::Imu>, &accel_checker);

    ros::spin();
    printf("\n");

    return 0;
}
