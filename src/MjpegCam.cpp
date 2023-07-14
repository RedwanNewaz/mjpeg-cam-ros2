#include "mjpeg_cam/MjpegCam.hpp"

namespace mjpeg_cam
{

void clamp(int &val, int min, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
}

MjpegCam::MjpegCam(const std::string &nodeName)
    : Node(nodeName),
      sequence(0)
{
    readParameters();
    CompressedImagePub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

    cam = std::make_unique<UsbCamera>(device_name, width, height);

    try {
        setCameraParams();
    }
    catch (const char * e){
        std::cout << e << std::endl;
    }

    RCLCPP_INFO(get_logger(), "Successfully launched node.");
    int milliseconds = (1 / framerate) * 1000;
    std::chrono::milliseconds duration(milliseconds);
    timer_ = this->create_wall_timer(duration, [this] { readAndPublishImage(); });
}

MjpegCam::~MjpegCam()
{

}

void MjpegCam::readAndPublishImage()
{
    try {
        sensor_msgs::msg::CompressedImage msg;
        msg.header.frame_id  = frame_id;
        msg.header.stamp = this->get_clock()->now();
        msg.format  = "jpeg";


        int length;
        auto image = cam->grab_image(length);
//        pubImage(image, length);
        msg.data.resize(length);
        std::copy(image, image + length, msg.data.begin());
//        RCLCPP_INFO(get_logger(), "length = %d KB", length / 1000);
        // Fill in the CompressedImage message
        CompressedImagePub_->publish(msg);

    }
    catch (const char *e) {
        std::cout << e << std::endl;
    }

}


void MjpegCam::readParameters()
{

    this->declare_parameter("device_name", "/dev/video0");
    this->declare_parameter("frame_id", "usb_cam");

    this->declare_parameter("width", 3840);
    this->declare_parameter("height", 2160);

    this->declare_parameter("exposure", 128);
    this->declare_parameter("brightness", 128);
    this->declare_parameter("framerate", 60);
    this->declare_parameter("autoexposure", true);

    device_name = this->get_parameter("device_name").get_parameter_value().get<std::string >();
    frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string >();

    width = this->get_parameter("width").get_parameter_value().get<int>();
    height = this->get_parameter("height").get_parameter_value().get<int>();

    exposure = this->get_parameter("exposure").get_parameter_value().get<int>();
    brightness = this->get_parameter("brightness").get_parameter_value().get<int>();
    framerate = this->get_parameter("framerate").get_parameter_value().get<int>();
    autoexposure = this->get_parameter("autoexposure").get_parameter_value().get<bool>();
}

bool MjpegCam::setCameraParams()
{


    clamp(exposure, 0, 255);
    clamp(brightness, 0, 255);

    cam->set_v4l2_param("brightness", brightness);

    if (autoexposure) {
        cam->set_v4l2_param("exposure_auto", 3);
    }
    else {
        cam->set_v4l2_param("exposure_auto", 1);
        cam->set_v4l2_param("exposure_absolute", exposure);
    }

//    cam->set_v4l2_param("power_line_frequency", 2);


    return true;
}

void MjpegCam::setDynamicParams(int exposure, int brightness, bool autoexposure)
{
    this->exposure = exposure;
    this->brightness = brightness;
    this->autoexposure = autoexposure;
    setCameraParams();
}



} /* namespace */
