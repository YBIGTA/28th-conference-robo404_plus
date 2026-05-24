#pragma once
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "yolo_bridge/common.hpp"

using namespace det;

class YOLOv8 {
private:
#ifdef USE_CUDA
    nvinfer1::ICudaEngine*       engine  = nullptr;
    nvinfer1::IRuntime*          runtime = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t                 stream  = nullptr;
    det::Logger                  gLogger{nvinfer1::ILogger::Severity::kERROR};
#else
    cv::dnn::Net net;
    std::vector<cv::Mat> dnn_outputs;
#endif

public:
    int                  num_bindings = 0;
    int                  num_inputs   = 0;
    int                  num_outputs  = 0;
    std::vector<void*>   host_ptrs;
    std::vector<void*>   device_ptrs;

    PreParam pparam;

#ifdef USE_CUDA
    std::vector<Binding> input_bindings;
    std::vector<Binding> output_bindings;
#endif

    explicit YOLOv8(const std::string& engine_file_path);
    ~YOLOv8();
    void MakePipe(bool warmup = true);
    void CopyFromMat(const cv::Mat& image);
    void CopyFromMat(const cv::Mat& image, cv::Size& size);
    void Letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size);
    void Infer();
    void PostProcess(std::vector<Object>& objs, float score_thres, float iou_thres, int topk, int num_labels = 80);
    void DrawObjects(cv::Mat& bgr, const std::vector<Object>& objs);
};