#include <types/PointClouds.hh>
#include <types/Yolo.hh>
#include <functions/progress_bar.hh>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ml.hpp>


cv::Mat image_from_cloud(linkml::PointCloud cloud){
    cv::Mat image = cv::Mat::zeros(cloud.height, cloud.width, CV_8UC3);
    #pragma omp parallel for shared(cloud, image)
    for (size_t i = 0; i < cloud.size(); i++){
        auto point = cloud.at(i);
        size_t x = i % cloud.width;
        size_t y = i / cloud.width;
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(point.b, point.g, point.r);
    }
    return image;
}


namespace linkml
{
    template<class T>
    static typename PointClouds<T>::Ptr annotate(typename PointClouds<T>::Ptr clouds, std::string yolo_path){ 

        auto model = Yolov8Seg(yolo_path, false);
        std::chrono::nanoseconds duration = std::chrono::nanoseconds(0);
        size_t n_frames = clouds->size();

        // Checks
        assert(n_frames > 0);
        for (size_t i = 0; i < n_frames; i++){
            if (!clouds->at(i).is_dense)
                throw std::runtime_error("Clouds must be dense");
        }




        std::vector<cv::Mat> blobs;
        blobs.resize(n_frames);
        std::vector<cv::Vec4d> params;
        params.resize(n_frames);
        std::vector<std::vector<cv::Mat>> outputs;
        outputs.resize(n_frames);
        std::vector<cv::Mat> source_images;
        source_images.resize(n_frames);


        // Factering out 
        auto infrence_precrocessing_bar = util::progress_bar(n_frames,"Preprocessing");
        #pragma omp parallel for shared(clouds, blobs, params)
        for (size_t i = 0; i < n_frames; i++){

            auto image = image_from_cloud(clouds->at(i));

            cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE); // <- Image in video is sideways

            source_images[i] = image;
            
            Yolov8Seg::Preprocess(image, blobs[i], params[i]);
            infrence_precrocessing_bar.update();
        }
        duration += infrence_precrocessing_bar.stop();

        // The neural network is not thread safe
        // _At least I am not able to make it thread safe_
        auto inference_bar = util::progress_bar(n_frames,"Running Inference");
        for (size_t i = 0; i < n_frames; i++){
            outputs[i] = model.Detect(blobs[i]);
            inference_bar.update();
        }
        duration +=  inference_bar.stop();

        auto inference_postprocessing_bar = util::progress_bar(n_frames,"Postprocessing");
        #pragma omp parallel for shared(clouds, outputs, params)
        for (size_t i = 0; i < n_frames; i++){

            auto results = Yolov8Seg::Postprocess(outputs[i], params[i], source_images[i]);

            // FIXME: Those values are wrong
            auto color_size = cv::Size(256,256); //dataset.color_size();
            auto depth_size = cv::Size(256,256); //dataset.depth_size();
            
            auto h = color_size.height;
            auto w = color_size.width;
            auto color_size_right_side_up = cv::Size(h, w);

            for (OutputParams const& param: results){

                auto param_rotated = param.Rotate<cv::ROTATE_90_COUNTERCLOCKWISE>(color_size_right_side_up).Scale(color_size, depth_size);

                auto row_start = param_rotated.box.y;
                auto row_end = param_rotated.box.y + param_rotated.box.height;

                auto col_start = param_rotated.box.x;
                auto col_end = param_rotated.box.x + param_rotated.box.width;

                #pragma omp parallel for collapse(2) shared(clouds, param_rotated)
                for (int row = row_start; row < row_end; row++){
                    for (int col = col_start; col < col_end; col++){
                        size_t index = row * depth_size.width + col;

                        if (param_rotated.boxMask.at<uchar>(row - row_start, col - col_start) > 0.1)
                            clouds[i]->points[index].semantic = param_rotated.id;
                    }
                }

            }
            
            inference_postprocessing_bar.update();
        }
        duration +=  inference_postprocessing_bar.stop();


        blobs.clear();
        params.clear();
        outputs.clear();
        source_images.clear();

    }

} // namespace linkml
