#include "chrono"
#include "opencv2/opencv.hpp"
#include "yolo_bridge/yolov8.hpp"

// ROS 2 核心头文件
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

// cv_bridge: 把 ROS Image 消息转成 OpenCV Mat
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;
using namespace det;

class YoloBridgeNode : public rclcpp::Node {
public:
    YoloBridgeNode(const string& engine_path, const string& input_source) 
    : Node("yolo_bridge_node"), engine_file_path(engine_path) {
        
        // 1. 创建发布者：发布速度指令给 linorobot2 和 yolo 检测结果
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        det_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("yolo/detections", 10);
        // 发布带标注的图像，方便用 rqt_image_view 查看
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/inference_result", 1);
        
        // 2. 初始化 YOLO (自动选择引擎)
#ifdef USE_CUDA
        cout << "Running on JETSON (CUDA Mode)" << endl;
        cudaSetDevice(0);
#else
        cout << "Running on VM (CPU Mode with OpenCV DNN)" << endl;
#endif
        yolov8_ = new YOLOv8(engine_file_path);
        yolov8_->MakePipe(true);

        // 3. 决定输入来源
        if (input_source == "0" || input_source == "camera") {
            // 打开本地 USB 摄像头
            cap_.open(0);
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");
            }
            use_camera_ = true;
            use_ros_topic_ = false;
            // 用定时器驱动推理循环
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33), std::bind(&YoloBridgeNode::run_inference_camera, this));
        } else if (input_source.find("/") == 0 || input_source == "rgb_cam/image_raw" 
                   || input_source == "rear_rgb_camera/image" || input_source.find("image") != string::npos) {
            // 订阅 ROS 话题 (Gazebo 虚拟摄像头)
            use_camera_ = false;
            use_ros_topic_ = true;
            RCLCPP_INFO(this->get_logger(), "Subscribing to ROS camera topic: %s", input_source.c_str());
            img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                input_source, 10,
                std::bind(&YoloBridgeNode::ros_image_callback, this, std::placeholders::_1));
        } else {
            // 读取静态图片文件
            use_camera_ = false;
            use_ros_topic_ = false;
            imagepath = input_source;
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33), std::bind(&YoloBridgeNode::run_inference_static, this));
        }
    }

    ~YoloBridgeNode() {
        if(yolov8_) delete yolov8_;
    }

private:
    // ============ 模式 1: 从 ROS 话题 (Gazebo) 获取图像 ============
    void ros_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image;
        try {
            // 用 cv_bridge 把 ROS Image 消息转成 OpenCV BGR Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            image = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "cv_bridge exception: %s", e.what());
            return;
        }

        if (image.empty()) return;

        // 运行 YOLO 推理
        std::vector<Object> objs;
        if (yolov8_) {
            cv::Size sz(640, 640);
            yolov8_->CopyFromMat(image, sz);
            yolov8_->Infer();
            yolov8_->PostProcess(objs, 0.25f, 0.65f, 100, 80);
        }

        // 处理动作 & 发布检测结果
        process_action(objs);

        // 绘制检测结果并发布标注图像
        if (yolov8_) yolov8_->DrawObjects(image, objs);
        publish_annotated_image(image);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Detected %zu objects from ROS topic", objs.size());
    }

    // ============ 模式 2: 从 USB 摄像头获取图像 ============
    void run_inference_camera() {
        Mat image;
        if (cap_.isOpened()) {
            cap_ >> image;
        }
        if (image.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to capture frame from camera!");
            return;
        }

        std::vector<Object> objs;
        if (yolov8_) {
            cv::Size sz(640, 640);
            yolov8_->CopyFromMat(image, sz);
            yolov8_->Infer();
            yolov8_->PostProcess(objs, 0.25f, 0.65f, 100, 80);
        }

        process_action(objs);
        if (yolov8_) yolov8_->DrawObjects(image, objs);
        publish_annotated_image(image);
    }

    // ============ 模式 3: 从静态图片文件获取图像 ============
    void run_inference_static() {
        Mat image = imread(imagepath);
        if (image.empty()) {
#ifndef USE_CUDA
            image = Mat::zeros(480, 640, CV_8UC3);
#else
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Image empty or not found!");
            return;
#endif
        }

        std::vector<Object> objs;
        if (yolov8_) {
            cv::Size sz(640, 640);
            yolov8_->CopyFromMat(image, sz);
            yolov8_->Infer();
            yolov8_->PostProcess(objs, 0.25f, 0.65f, 100, 80);
        }

        process_action(objs);
        if (yolov8_) yolov8_->DrawObjects(image, objs);
        publish_annotated_image(image);
    }

    // ============ 发布带标注的图像到 /inference_result ============
    void publish_annotated_image(const cv::Mat& image) {
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->header.frame_id = "camera_link";
        img_pub_->publish(*img_msg);
    }

    // ============ 发布检测结果 + 控制指令 ============
    void process_action(const std::vector<Object>& objs) {
        // --- 1. 发布 Detection2DArray ---
        auto det_array = vision_msgs::msg::Detection2DArray();
        det_array.header.stamp = this->get_clock()->now();
        det_array.header.frame_id = "camera_link";

        for (const auto& obj : objs) {
            vision_msgs::msg::Detection2D det;
            det.header = det_array.header;
            
            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.id = std::to_string(obj.label);
            hyp.score = obj.prob;
            det.results.push_back(hyp);
            
            det.bbox.center.x = obj.rect.x + obj.rect.width / 2.0;
            det.bbox.center.y = obj.rect.y + obj.rect.height / 2.0;
            det.bbox.center.theta = 0.0;
            det.bbox.size_x = obj.rect.width;
            det.bbox.size_y = obj.rect.height;
            
            det_array.detections.push_back(det);
        }
        det_pub_->publish(det_array);

        // --- 2. 原有的 Twist 逻辑 ---
        auto twist = geometry_msgs::msg::Twist();
        
        if (objs.empty()) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        } else {
            // 简单的"视觉寻迹"逻辑：
            // 计算物体中心点 x 坐标
            float obj_center_x = objs[0].rect.x + objs[0].rect.width / 2.0;
            float img_center_x = 320.0; // 假设 640 宽

            // 计算误差：物体在中心左边还是右边
            float error = img_center_x - obj_center_x;

            // 产生动作：根据误差转向
            twist.angular.z = error * 0.005; 
            
            // 距离控制：如果物体在画面中很小（离得远），就前进
            if (objs[0].rect.width < 200) {
                twist.linear.x = 0.2; 
            } else {
                twist.linear.x = 0.0; // 离近了停车
            }
        }
        // 发布指令 (commented out so keyboard teleop works)
        // cmd_pub_->publish(twist);
    }

    string engine_file_path;
    string imagepath;
    bool use_camera_;
    bool use_ros_topic_;
    cv::VideoCapture cap_;
    YOLOv8* yolov8_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    if (argc < 3) {
        fprintf(stderr, "Usage: ros2 run yolo_bridge yolo_node <model.onnx> <input_source>\n");
        fprintf(stderr, "  input_source can be:\n");
        fprintf(stderr, "    - A ROS topic name (e.g. 'rgb_cam/image_raw' or '/rear_rgb_camera/image')\n");
        fprintf(stderr, "    - '0' or 'camera' for USB camera\n");
        fprintf(stderr, "    - A path to an image file\n");
        return -1;
    }
    auto node = std::make_shared<YoloBridgeNode>(argv[1], argv[2]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}