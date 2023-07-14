#pragma once

#include "mjpeg_cam/UsbCamera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>

namespace mjpeg_cam
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
    class MjpegCam: public rclcpp::Node
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    MjpegCam(const std::string &nodeName);

    /*!
     * Destructor.
     */
    virtual ~MjpegCam();



    /*!
     * Set parameters that can be dynamically reconfigured
     */
    void setDynamicParams(int exposure, int brightness, bool autoexposure);

private:
    /*!
     * Reads a single frame from the camera and publish to topic.
     */
    void readAndPublishImage();

    /*!
     * Reads ROS parameters.
     */
    void readParameters();

    /*!
     * Set camera parameters
     */
    bool setCameraParams();

    void pubImage(const uchar *image, int length);


    //! ROS Image Publisher

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr CompressedImagePub_;
    rclcpp::TimerBase::SharedPtr timer_;
    //! Camera Object
    std::unique_ptr<UsbCamera> cam;
    unsigned int sequence;

    // Parameters
    std::string device_name;
    std::string frame_id;
    int width;
    int height;
    int framerate;
    int exposure;
    int brightness;
    bool autoexposure;
};

} /* namespace */
