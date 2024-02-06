#pragma once
#include <vector>
#include <exception>
#include <optional>
#include <opencv4/opencv2/opencv.hpp>


namespace linkml{

    //https://github.com/UNeedCryDear/yolov8-opencv-onnxruntime-cpp

    struct OutputParams {
        int id;                     //Result category id
        float confidence;           //Result confidence
        cv::Rect box;               //Rectangle
        cv::RotatedRect rotatedBox; //obb result rectangular box
        cv::Mat boxMask;            //Mask within the rectangular frame to save memory space and speed up

        template <cv::RotateFlags flag>
        OutputParams Rotate(cv::Size size) const;

        OutputParams Scale(cv::Size size_in, cv::Size size_out) const {
            OutputParams result;
            result.id = id;
            result.confidence = confidence;

            double scale_x = (double)size_out.width / size_in.width;
            double scale_y = (double)size_out.height / size_in.height;

            //rect [x,y,w,h]
            result.box =  cv::Rect(box.x * scale_x, box.y * scale_y, box.width * scale_x, box.height * scale_y);

            //rect [x,y,w,h]
            cv::Point center = cv::Point(rotatedBox.center.x * scale_x, rotatedBox.center.y * scale_y);
            cv::Size size = cv::Size(0,0);
            result.rotatedBox =  cv::RotatedRect(center,size, rotatedBox.angle);

            cv::Size size_target = cv::Size(boxMask.cols * scale_x, boxMask.rows * scale_y);
            cv::resize(boxMask, result.boxMask, size_target);

            return result;
        }

    };
    struct MaskParams {
        //int segChannels = 32;
        //int segWidth = 160;
        //int segHeight = 160;
        int netWidth = 640;
        int netHeight = 640;
        float maskThreshold = 0.5;
        cv::Size srcImgShape;
        cv::Vec4d params;
    };

    class Yolov8Seg
    {
    private:
        inline static const std::vector<std::string> _className = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
            "stop sign","parking meter","bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
            "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball","kite", "baseball bat",
            "baseball glove","skateboard", "surfboard", "tennis racket","bottle", "wine glass","cup", "fork", "knife", "spoon", "bowl",
            "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog","pizza", "donut", "cake", "chair", "couch",
            "potted plant","bed", "dining table","toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone","microwave",
            "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase","scissors", "teddy bear","hair drier","toothbrush"
        };

        cv::dnn::Net model;

        int _netWidth = 640;
        int _netHeight = 640;

        float _classThreshold   = 0.25;
        float _nmsThreshold     = 0.45;
        float _maskThreshold    = 0.5;

    public:
        Yolov8Seg(std::string netPath, bool isCuda = false) {

            model = cv::dnn::readNetFromONNX(netPath);
            if (isCuda) {
                model.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                model.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); //or DNN_TARGET_CUDA_FP16
            }
            else {
                model.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
                model.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
        };

        std::optional<std::vector<OutputParams>> Detect(cv::Mat srcImg);

        static std::string GetClassName(int id) { return _className[id];}
    
    };
}