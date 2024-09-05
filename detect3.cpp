#include <iostream>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

constexpr int IMAGE_WIDTH = 1280;
constexpr int IMAGE_HEIGHT = 1024;

// 模型输入和输出处理函数
cv::Mat preprocessImage(const cv::Mat& image) {
    cv::Mat processedImage;
    cv::resize(image, processedImage, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
    // 数据增强
    cv::RNG rng;
    if (rng.uniform(0, 2) == 0) {
        cv::flip(processedImage, processedImage, 1);  // 水平翻转
    }
    if (rng.uniform(0, 2) == 0) {
        cv::rotate(processedImage, processedImage, cv::ROTATE_90_CLOCKWISE);  // 顺时针旋转 90 度
    }
    // 归一化
    processedImage.convertTo(processedImage, CV_32FC3, 1.0 / 255.0);
    return processedImage;
}
//提取检测结果、计算位置和姿态信息
void postprocessOutput(Ort::Value& outputTensor, cv::Mat& image, const std::vector<cv::Point3f>& buff_3d_points) {
    auto outputTensorData = outputTensor.GetTensorMutableData<float>();

    // 提取检测框信息、分类置信度和关键点坐标
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;
    std::vector<std::vector<cv::Point2f>> keypoints;

    // 解析模型输出，填充上述数据结构

    // 置信度阈值过滤
    float confidenceThreshold = 0.5;  // 可根据实际情况调整
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold, 0.4, indices);

    // 只处理过滤后的结果
    std::vector<cv::Rect> filteredBoxes;
    std::vector<float> filteredConfidences;
    std::vector<int> filteredClassIds;
    std::vector<std::vector<cv::Point2f>> filteredKeypoints;

    for (int index : indices) {
        filteredBoxes.push_back(boxes[index]);
        filteredConfidences.push_back(confidences[index]);
        filteredClassIds.push_back(classIds[index]);
        filteredKeypoints.push_back(keypoints[index]);
    }

    // 计算 PnP 求解位置和姿态
    std::vector<cv::Point2f> imagePoints; 
    std::vector<cv::Point3f> objectPoints;

    // 根据过滤后的检测结果填充 imagePoints 和 objectPoints

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cv::Mat(camera_matrix), cv::Mat(distortion_coefficients), rvec, tvec);

    // 输出旋转向量和平移向量
    std::cout << "Rotation Vector: " << rvec << std::endl;
    std::cout << "Translation Vector: " << tvec << std::endl;
}

int main() {
    // 初始化 OnnxRuntime 环境
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLOv8");
    Ort::SessionOptions sessionOptions;

    // 加载模型
    Ort::Session session(env, "best.onnx", sessionOptions);

    cv::VideoCapture video("test_video.mp4");

    if (!video.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
        return -1;
    }

    cv::Mat frame;

    while (video.read(frame)) {
        // 预处理图像
        cv::Mat processedFrame = preprocessImage(frame);

        // 准备输入
        std::vector<int64_t> inputDims = {1, 3, IMAGE_HEIGHT, IMAGE_WIDTH};
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memoryInfo, processedFrame.data, processedFrame.total() * processedFrame.elemSize(), inputDims.data(), inputDims.size());

        // 执行推理
        std::vector<Ort::Value> outputTensors = session.Run(Ort::RunOptions{nullptr}, {"input"}, &inputTensor, 1);
        std::vector<cv::Point3f> buff_3d_points = {
          {0, 0.1700, 0.1750},
          {0, -0.1700, 0.1750},
          {0, -0.1850, -0.1650},
          {0, 0, -0.7150},
          {0, 0.1850, -0.1650}
          };

         while (video.read(frame)) {
        postprocessOutput(outputTensors[0], frame, buff_3d_points);

         }

        // 显示结果或进行其他处理
        cv::imshow("Detection Result", frame);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    video.release();
    cv::destroyAllWindows();

    return 0;
}
