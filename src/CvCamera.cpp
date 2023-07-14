//
// Created by redwan on 7/14/23.
//

#include "mjpeg_cam/CvCamera.h"

CvCamera::CvCamera(const std::string &device, int width, int height): cap_thread_() {

    // Define the regular expression pattern to match an integer
    std::regex pattern("(\\d+)");
    // Define a match object to store the matched results
    std::smatch match;
    // Search for the first integer match in the string
    assert (std::regex_search(device, match, pattern));
    // Access the matched integer value
    int deviceID = std::stoi(match[1]);
    cap_ = std::make_unique<cv::VideoCapture>(deviceID);

    frameArrived_ = false;

    assert(cap_->isOpened());


//// FIXME setting up this fails the camera to be operated

////    // Set the MJPEG camera.
//    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
//    cap_->set(cv::CAP_PROP_FOURCC, fourcc);
//
////    // Set the image size.
//    cap_->set(cv::CAP_PROP_FRAME_WIDTH, width);
//    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, height);

    cap_thread_ = std::thread(&CvCamera::run, this);

}

void CvCamera::set_v4l2_param(const std::string &param, int value) {

}

char *CvCamera::grab_image(int &len) {


    bool ret = !frame.empty();
    if(!ret)
    {
        std::cerr << "could not capture any images" << std::endl;
        return nullptr;
    }

    std::vector<uchar>raw_data;
    cv::imencode(".jpg", frame, raw_data);

    len = 203000;
    return reinterpret_cast<char*>(raw_data.data());
}

CvCamera::~CvCamera() {
    cap_->release();
    if(cap_thread_.joinable())
        cap_thread_.join();
}

void CvCamera::run() {
    while (true) {
        // Capture a frame.
        bool ret = cap_->read(frame);
        if (!ret)
            continue;
        frameArrived_ = true;
    }

}

bool CvCamera::isImageAvailable() {
    return frameArrived_;
}

cv::Mat CvCamera::getImg() {
    frameArrived_ = false;
    return frame;
}
