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
            "person",          // 0
            "bicycle",         // 1
            "car",             // 2
            "motorcycle",      // 3
            "airplane",        // 4
            "bus",             // 5
            "train",           // 6
            "truck",           // 7
            "boat",            // 8
            "traffic light",   // 9
            "fire hydrant",    // 10
            "stop sign",       // 11
            "parking meter",   // 12
            "bench",           // 13
            "bird",            // 14
            "cat",             // 15
            "dog",             // 16
            "horse",           // 17
            "sheep",           // 18
            "cow",             // 19
            "elephant",        // 20
            "bear",            // 21
            "zebra",           // 22
            "giraffe",         // 23
            "backpack",        // 24
            "umbrella",        // 25
            "handbag",         // 26
            "tie",             // 27
            "suitcase",        // 28
            "frisbee",         // 29
            "skis",            // 30
            "snowboard",       // 31
            "sports ball",     // 32
            "kite",            // 33
            "baseball bat",    // 34
            "baseball glove",  // 35
            "skateboard",      // 36
            "surfboard",       // 37
            "tennis racket",   // 38
            "bottle",          // 39
            "wine glass",      // 40
            "cup",             // 41
            "fork",            // 42
            "knife",           // 43
            "spoon",           // 44
            "bowl",            // 45
            "banana",          // 46
            "apple",           // 47
            "sandwich",        // 48
            "orange",          // 49
            "broccoli",        // 50
            "carrot",          // 51
            "hot dog",         // 52
            "pizza",           // 53
            "donut",           // 54
            "cake",            // 55
            "chair",           // 56
            "couch",           // 57
            "potted plant",    // 58
            "bed",             // 59
            "dining table",    // 60
            "toilet",          // 61
            "tv",              // 62
            "laptop",          // 63
            "mouse",           // 64
            "remote",          // 65
            "keyboard",        // 66
            "cell phone",      // 67
            "microwave",       // 68
            "oven",            // 69
            "toaster",         // 70
            "sink",            // 71
            "refrigerator",    // 72
            "book",            // 73
            "clock",           // 74
            "vase",            // 75
            "scissors",        // 76
            "teddy bear",      // 77
            "hair drier",      // 78
            "toothbrush"       // 79
        };

        cv::dnn::Net model;

        static int _netWidth;
        static int _netHeight;

        static float _classThreshold;
        static float _nmsThreshold;
        static float _maskThreshold;

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

        std::vector<cv::Mat> Detect(cv::Mat srcImg);

        static void Preprocess(cv::Mat & srcImg, cv::Mat & blob, cv::Vec4d & params);
        static std::vector<OutputParams> Postprocess(const std::vector<cv::Mat> & blob, const cv::Vec4d & params, const cv::Mat & srcImg);


        static std::string GetClassName(int id) { return _className[id];}
    
    };
}