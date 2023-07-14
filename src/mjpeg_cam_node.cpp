#include <rclcpp/rclcpp.hpp>
#include "mjpeg_cam/MjpegCam.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto MjpegCam = std::make_shared<mjpeg_cam::MjpegCam> ("MjpegCam");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(MjpegCam);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
