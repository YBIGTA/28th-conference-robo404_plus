#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

class CsiCameraNode : public rclcpp::Node
{
public:
    CsiCameraNode()
        : Node("csi_camera_node")
    {
        sensor_id_ = this->declare_parameter<int>("sensor_id", 0);
        capture_width_ = this->declare_parameter<int>("capture_width", 1280);
        capture_height_ = this->declare_parameter<int>("capture_height", 720);
        output_width_ = this->declare_parameter<int>("output_width", 960);
        output_height_ = this->declare_parameter<int>("output_height", 540);
        framerate_ = this->declare_parameter<int>("framerate", 30);
        flip_method_ = this->declare_parameter<int>("flip_method", 0);
        frame_id_ = this->declare_parameter<std::string>("frame_id", "camera");

        validate_parameters();
        open_capture();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS());

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / static_cast<double>(framerate_)));

        timer_ = this->create_wall_timer(
            period, std::bind(&CsiCameraNode::publish_frame, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing CSI camera sensor_id=%d as frame_id='%s'",
            sensor_id_,
            frame_id_.c_str());
    }

    ~CsiCameraNode() override
    {
        if (capture_.isOpened()) {
            capture_.release();
        }
    }

private:
    void validate_parameters() const
    {
        if (sensor_id_ < 0) {
            throw std::invalid_argument("sensor_id must be >= 0");
        }
        if (capture_width_ <= 0 || capture_height_ <= 0) {
            throw std::invalid_argument("capture_width and capture_height must be > 0");
        }
        if (output_width_ <= 0 || output_height_ <= 0) {
            throw std::invalid_argument("output_width and output_height must be > 0");
        }
        if (framerate_ <= 0) {
            throw std::invalid_argument("framerate must be > 0");
        }
        if (flip_method_ < 0 || flip_method_ > 7) {
            throw std::invalid_argument("flip_method must be in the Jetson nvvidconv range 0..7");
        }
        if (frame_id_.empty()) {
            throw std::invalid_argument("frame_id must not be empty");
        }
    }

    std::string make_pipeline(const std::string& sensor_property_name) const
    {
        std::ostringstream pipeline;
        pipeline
            << "nvarguscamerasrc "
            << sensor_property_name << "=" << sensor_id_
            << " ! video/x-raw(memory:NVMM), width=(int)" << capture_width_
            << ", height=(int)" << capture_height_
            << ", framerate=(fraction)" << framerate_ << "/1"
            << " ! nvvidconv flip-method=" << flip_method_
            << " ! video/x-raw, width=(int)" << output_width_
            << ", height=(int)" << output_height_
            << ", format=(string)BGRx"
            << " ! videoconvert"
            << " ! video/x-raw, format=(string)BGR"
            << " ! appsink drop=1 sync=false max-buffers=1";
        return pipeline.str();
    }

    void open_capture()
    {
        const std::vector<std::string> sensor_property_names = {
            "sensor_id",
            "sensor-id",
        };

        std::vector<std::string> attempted_pipelines;
        for (const auto& sensor_property_name : sensor_property_names) {
            const auto pipeline = make_pipeline(sensor_property_name);
            attempted_pipelines.push_back(pipeline);

            RCLCPP_INFO(
                this->get_logger(),
                "Opening CSI camera with pipeline: %s",
                pipeline.c_str());

            capture_.open(pipeline, cv::CAP_GSTREAMER);
            if (capture_.isOpened()) {
                selected_pipeline_ = pipeline;
                return;
            }

            capture_.release();
        }

        std::ostringstream error;
        error << "Failed to open CSI camera sensor_id=" << sensor_id_
              << ". Attempted pipelines:";
        for (const auto& pipeline : attempted_pipelines) {
            error << "\n  " << pipeline;
        }
        throw std::runtime_error(error.str());
    }

    void publish_frame()
    {
        cv::Mat frame;
        if (!capture_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to read frame from CSI camera sensor_id=%d",
                sensor_id_);
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = frame_id_;

        auto image_msg = cv_bridge::CvImage(
            header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();

        publisher_->publish(*image_msg);
    }

    int sensor_id_;
    int capture_width_;
    int capture_height_;
    int output_width_;
    int output_height_;
    int framerate_;
    int flip_method_;
    std::string frame_id_;
    std::string selected_pipeline_;

    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<CsiCameraNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("csi_camera_node"),
            "%s",
            error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
