#include <types/PointClouds.hh>
#include <types/Yolo.hh>
#include <types/Dataset.hh>

#include <functions/progress_bar.hh>
#include <functions/color.hh>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ml.hpp>

#include <optional>



static void draw_box(cv::Mat & img,  linkml::OutputParams const& param){
    auto color = linkml::get_color_forom_angle(linkml::sample_circle(param.id));
    cv::Scalar color_scalar = cv::Scalar(color.r * 255, color.g * 255, color.b * 255);
    std::string label = linkml::Yolov8Seg::GetClassName(param.id);
    cv::rectangle(img, param.box, color_scalar, 2);
    cv::putText(img, label, cv::Point(param.box.x, param.box.y), cv::FONT_HERSHEY_SIMPLEX, 1, color_scalar, 2);

    cv::Mat mask = param.boxMask;
    mask = ( mask > 0.5);
    mask.convertTo(mask, CV_8U);
    std::vector<cv::Mat> contours;
    cv::Mat hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat overlay = img.clone();
    cv::drawContours(overlay(param.box), contours, -1, color_scalar, cv::FILLED, cv::LINE_8, hierarchy,100);
    cv::addWeighted(overlay, 0.7, img, 0.3, 0, img);
}


namespace linkml
{
    template<class T>
    PointClouds<T> PointClouds<T>::annotate(std::string yolo_path, std::optional<Dataset> & dataset){ 


        auto model = Yolov8Seg(yolo_path, true); // <- true for GPU
        std::chrono::nanoseconds duration = std::chrono::nanoseconds(0);
        size_t n_frames = data.size();



        // Checks
        assert(n_frames > 0);
        if constexpr (std::is_same<T, std::string>::value){
            std::cout << "Not checking if point cloud is dense for On disk point clouds" << std::endl;
        } else if constexpr (std::is_same<T, PointCloud>::value){
            for (size_t i = 0; i < n_frames; i++){
                if (!data.at(i)->is_dense)
                    throw std::runtime_error("Clouds must be dense");
            }
        }


        // Load images
        std::vector<cv::Mat> input_images;
        input_images.resize(n_frames);
        auto load_images_bar = util::progress_bar(n_frames,"Loading Images");
        if (dataset.has_value()){
            #pragma omp parallel for shared(data, input_images)
            for (size_t i = 0; i < n_frames; i++){
                pcl::PCLHeader header;
                if constexpr (std::is_same<T, std::string>::value)
                    header = PointCloud::load_header(data.at(i));
                else if constexpr (std::is_same<T, PointCloud>::value){
                    header = data.at(i)->header;
                }
                
                size_t index = std::atoi(header.frame_id.c_str());
                input_images[i] = dataset.value()[index].get<Field::COLOR>();
                load_images_bar.update();
            }
        } else {
            #pragma omp parallel for shared(data, input_images)
            for (size_t i = 0; i < n_frames; i++){
                PointCloud cloud;
                if constexpr (std::is_same<T, std::string>::value)
                    cloud = PointCloud::load(data.at(i));
                else if constexpr (std::is_same<T, PointCloud>::value){
                    cloud = data.at(i);
                } 
                input_images[i] = cloud.image();
                load_images_bar.update();
            }
        }
        duration += load_images_bar.stop();




        // Preprocessing
        std::vector<cv::Mat> blobs;
        blobs.resize(n_frames);
        std::vector<cv::Vec4d> params;
        params.resize(n_frames);
        auto preprocessing_bar = util::progress_bar(n_frames,"Preprocessing");
        #pragma omp parallel for shared(input_images, blobs, params)
        for (size_t i = 0; i < n_frames; i++){
            cv::rotate(input_images[i], input_images[i], cv::ROTATE_90_CLOCKWISE);
            //cv::resize(input_images[i], input_images[i], cv::Size(640,640));
            Yolov8Seg::Preprocess(input_images[i], blobs[i], params[i]);
            preprocessing_bar.update();
        }
        duration += preprocessing_bar.stop();


        // Inference
        // The neural network is not thread safe
        // _At least I am not able to make it thread safe_
        std::vector<std::vector<cv::Mat>> outputs;
        outputs.resize(n_frames);
        auto inference_bar = util::progress_bar(n_frames,"Running Inference");
        for (size_t i = 0; i < n_frames; i++){
            outputs[i] = model.Detect(blobs[i]);
            inference_bar.update();
        }
        duration +=  inference_bar.stop();
        blobs.clear(); // <- Free memory

        //cv::destroyAllWindows();


        auto inference_postprocessing_bar = util::progress_bar(n_frames,"Postprocessing");
        #pragma omp parallel for shared(data, outputs, params, input_images)
        for (size_t i = 0; i < n_frames; i++){

            auto results = Yolov8Seg::Postprocess(outputs[i], params[i], input_images[i].size());

            

            PointCloud cloud;
            if constexpr (std::is_same<T, std::string>::value){
                cloud = PointCloud::load(data.at(i));
            } else if constexpr (std::is_same<T, PointCloud>::value){
                cloud = data.at(i);
            } 


            // Reset the semantic field
            #pragma omp parallel for
            for (size_t i =0; i < cloud->size(); i++){
                cloud->at(i).semantic = 0;
            }


            cv::Size cloud_size = cv::Size(cloud->width, cloud->height);
            //cv::Size input_image_size_right_side_up = input_images[i].size();// cv::Size(input_image_size.height, input_image_size.width);
            cv::Size input_image_size = cv::Size(input_images[i].size().width, input_images[i].size().height);


            //auto img = cloud->image();
            //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

            for (OutputParams const& result_params: results ){
                OutputParams result_param_r = result_params
                    .Scale(input_image_size, cv::Size(cloud_size.height, cloud_size.width))
                    .Rotate<cv::ROTATE_90_COUNTERCLOCKWISE>(cv::Size(cloud_size.height, cloud_size.width));

                if (!result_param_r.valid)
                    continue;
                //draw_box(img, result_param_r);

                // Get the start and end of the box
                const auto row_start = result_param_r.box.y;
                const auto row_end = result_param_r.box.y + result_param_r.box.height;
                const auto col_start = result_param_r.box.x;
                const auto col_end = result_param_r.box.x + result_param_r.box.width;
                // Annotate the point cloud
                #pragma omp parallel for collapse(2) shared(cloud, result_param_r) firstprivate(row_start, row_end, col_start, col_end)
                for (int y = row_start; y < row_end; y++){
                    for (int x = col_start; x < col_end; x++){
                        if (result_param_r.boxMask.at<uchar>(y - row_start, x - col_start) > 0.5)
                            cloud->at(x, y).semantic = result_param_r.id;
                    }
                }
            }


            //cv::Mat img = cloud->image("semantic");
            //cv::addWeighted(img, 0.7, cloud->image("semantic"), 0.3, 0, img);
            //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            //cv::imshow("Annotated Cloud", img);
            //cv::waitKey(0);

            if constexpr (std::is_same<T, std::string>::value)
                cloud.save(data.at(i));

            inference_postprocessing_bar.update();
        }
        duration += inference_postprocessing_bar.stop();




        blobs.clear();
        params.clear();
        outputs.clear();
        input_images.clear(); 

        // cv::destroyAllWindows();

        return *this;

       
    }

    template PointCloudsInMemory  PointCloudsInMemory::annotate(std::string ,std::optional<Dataset> & );
    template PointCloudsOnDisk  PointCloudsOnDisk::annotate(std::string ,std::optional<Dataset> & );

} // namespace linkml
