#ifndef _FACE_MOBILEFACENET_H_
#define _FACE_MOBILEFACENET_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "Interpreter.hpp"
#include "ImageProcess.hpp"
#include "MNNDefine.h"
#include "Tensor.hpp"

class Mobilefacenet{
public:
    Mobilefacenet(const std::string& model_file);
    ~Mobilefacenet();
    int ExtractFeature(const cv::Mat& img_face, std::vector<float>* feat);

private:

    std::shared_ptr<MNN::Interpreter> mobilefacenet_interpreter_ = nullptr;
    MNN::Session* mobilefacenet_sess_ = nullptr;
    MNN::Tensor* input_tensor_ = nullptr;

    const cv::Size inputSize_ = cv::Size(112, 112);
    const float meanVals_[3] = {     127.5f,     127.5f,     127.5f };
    const float normVals_[3] = { 0.0078125f, 0.0078125f, 0.0078125f };
};

#endif // !_FACE_MOBILEFACENET_H_
