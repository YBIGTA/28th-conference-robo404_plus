#include <algorithm>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "yolo_msgs/msg/detection.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

#include "yolov8.hpp"

extern const char* class_names[];

namespace
{
bool file_exists(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    return file.good();
}

rmw_qos_reliability_policy_t reliability_from_int(int reliability)
{
    switch (reliability) {
        case 1:
            return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        case 2:
            return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        default:
            return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    }
}
}  // namespace

class YoloJetsonNode : public rclcpp::Node {
public:
    YoloJetsonNode()
        : Node("yolo_node")
    {
        engine_path_ = this->declare_parameter<std::string>("engine_path", "");
        yolo_encoding_ = this->declare_parameter<std::string>("yolo_encoding", "bgr8");
        enable_ = this->declare_parameter<bool>("enable", true);
        threshold_ = this->declare_parameter<double>("threshold", 0.5);
        iou_ = this->declare_parameter<double>("iou", 0.5);
        imgsz_height_ = this->declare_parameter<int>("imgsz_height", 640);
        imgsz_width_ = this->declare_parameter<int>("imgsz_width", 640);
        max_det_ = this->declare_parameter<int>("max_det", 100);
        num_labels_ = this->declare_parameter<int>("num_labels", 80);
        const int image_reliability = this->declare_parameter<int>("image_reliability", 2);

        if (engine_path_.empty()) {
            throw std::runtime_error("Parameter 'engine_path' is required");
        }
        if (!file_exists(engine_path_)) {
            throw std::runtime_error("TensorRT engine file does not exist: " + engine_path_);
        }
        if (imgsz_width_ <= 0 || imgsz_height_ <= 0) {
            throw std::runtime_error("imgsz_width and imgsz_height must be positive");
        }
        if (max_det_ <= 0) {
            throw std::runtime_error("max_det must be positive");
        }
        if (num_labels_ <= 0 || num_labels_ > 80) {
            throw std::runtime_error("num_labels must be in the range [1, 80]");
        }

        rclcpp::QoS image_qos(rclcpp::KeepLast(1));
        image_qos.reliability(reliability_from_int(image_reliability));

        pub_ = this->create_publisher<yolo_msgs::msg::DetectionArray>("detections", 10);
        enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "enable",
            std::bind(
                &YoloJetsonNode::enable_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Loading TensorRT engine: %s", engine_path_.c_str());
        cudaSetDevice(0);
        yolo_ = std::make_unique<YOLOv8>(engine_path_);
        yolo_->MakePipe(true);
        input_size_ = cv::Size(imgsz_width_, imgsz_height_);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            image_qos,
            std::bind(&YoloJetsonNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TensorRT YOLO node is ready");
    }

private:
    void enable_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        enable_ = request->data;
        response->success = true;
        response->message = enable_ ? "YOLO enabled" : "YOLO disabled";
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        yolo_msgs::msg::DetectionArray detections_msg;
        detections_msg.header = msg->header;

        if (!enable_) {
            pub_->publish(detections_msg);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, yolo_encoding_);
        } catch (const cv_bridge::Exception& error) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "cv_bridge conversion failed: %s",
                error.what());
            pub_->publish(detections_msg);
            return;
        }

        if (cv_ptr->image.empty()) {
            pub_->publish(detections_msg);
            return;
        }

        std::vector<det::Object> objects;
        yolo_->CopyFromMat(cv_ptr->image, input_size_);
        yolo_->Infer();
        yolo_->PostProcess(
            objects,
            static_cast<float>(threshold_),
            static_cast<float>(iou_),
            max_det_,
            num_labels_);

        detections_msg.detections.reserve(objects.size());
        for (const auto& object : objects) {
            if (object.label < 0 || object.label >= num_labels_) {
                continue;
            }
            detections_msg.detections.push_back(to_detection_msg(object));
        }

        pub_->publish(detections_msg);
    }

    yolo_msgs::msg::Detection to_detection_msg(const det::Object& object) const
    {
        yolo_msgs::msg::Detection detection;

        detection.class_id = object.label;
        detection.class_name = class_names[object.label];
        detection.score = object.prob;

        detection.bbox.center.position.x = object.rect.x + object.rect.width / 2.0f;
        detection.bbox.center.position.y = object.rect.y + object.rect.height / 2.0f;
        detection.bbox.center.theta = 0.0;
        detection.bbox.size.x = object.rect.width;
        detection.bbox.size.y = object.rect.height;

        return detection;
    }

    std::string engine_path_;
    std::string yolo_encoding_;
    bool enable_;
    double threshold_;
    double iou_;
    int imgsz_height_;
    int imgsz_width_;
    int max_det_;
    int num_labels_;
    cv::Size input_size_;
    std::unique_ptr<YOLOv8> yolo_;

    rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<YoloJetsonNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("yolo_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
