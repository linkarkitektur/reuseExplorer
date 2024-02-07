#include "Yolo.hh"
#include <vector>
#include <optional>
#include <opencv4/opencv2/opencv.hpp>


namespace linkml {


    template <>
    OutputParams OutputParams::Rotate<cv::RotateFlags::ROTATE_90_CLOCKWISE>(cv::Size size) const {
        OutputParams result;
        result.id = id;
        result.confidence = confidence;

        //rect [x,y,w,h]
        result.box = cv::Rect( size.height - box.y - box.height, box.x,   box.height, box.width);

        //rect [x,y,w,h]
        cv::Point center = cv::Point(size.width - rotatedBox.center.y , rotatedBox.center.x);
        result.rotatedBox = cv::RotatedRect( center, rotatedBox.size, rotatedBox.angle + 90);

        cv::Mat rt_boxMask;
        cv::rotate(boxMask,result.boxMask, cv::RotateFlags::ROTATE_90_CLOCKWISE);

        return result;
    }

    template <>
    OutputParams OutputParams::Rotate<cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE>(cv::Size size) const {
        OutputParams result;
        result.id = id;
        result.confidence = confidence;

        //rect [x,y,w,h]
        result.box = cv::Rect( box.y, size.width - box.x - box.width,   box.height, box.width);

        //rect [x,y,w,h]
        cv::Point center = cv::Point(rotatedBox.center.y, size.height - rotatedBox.center.x);
        result.rotatedBox = cv::RotatedRect(  center, rotatedBox.size, rotatedBox.angle - 90);

        cv::Mat rt_boxMask;
        cv::rotate(boxMask,result.boxMask, cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE);

        return result;
    }

    template <>
    OutputParams OutputParams::Rotate<cv::RotateFlags::ROTATE_180>(cv::Size size) const {
        OutputParams result;
        result.id = id;
        result.confidence = confidence;

        //rect [x,y,w,h]
        result.box =  cv::Rect(size.height - box.y - box.height, size.width - box.x - box.width, box.width, box.height);

        //rect [x,y,w,h]
        cv::Point center = cv::Point(size.width - rotatedBox.center.x, size.height - rotatedBox.center.y);
        result.rotatedBox =  cv::RotatedRect( center, rotatedBox.size, rotatedBox.angle);

        cv::Mat rt_boxMask;
        cv::rotate(boxMask,result.boxMask, cv::RotateFlags::ROTATE_180);

        return result;
    }



    // void LetterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params, const cv::Size& newShape,
    //    bool autoShape, bool scaleFill, bool scaleUp, int stride, const cv::Scalar& color)

    void LetterBox(const cv::Mat& image, cv::Mat& outImage,
	cv::Vec4d& params, //[ratio_x,ratio_y,dw,dh]
	const cv::Size& newShape = cv::Size(640, 640),
	bool autoShape = false,
	bool scaleFill = false,
	bool scaleUp = true,
	int stride = 32,
	const cv::Scalar& color = cv::Scalar(114, 114, 114))
    {
        if (false) {
            int maxLen = MAX(image.rows, image.cols);
            outImage = cv::Mat::zeros(cv::Size(maxLen, maxLen), CV_8UC3);
            image.copyTo(outImage(cv::Rect(0, 0, image.cols, image.rows)));
            params[0] = 1;
            params[1] = 1;
            params[3] = 0;
            params[2] = 0;
        }

        cv::Size shape = image.size();
        float r = std::min((float)newShape.height / (float)shape.height,
            (float)newShape.width / (float)shape.width);
        if (!scaleUp)
            r = std::min(r, 1.0f);

        float ratio[2]{ r, r };
        int new_un_pad[2] = { (int)std::round((float)shape.width * r),(int)std::round((float)shape.height * r) };

        auto dw = (float)(newShape.width - new_un_pad[0]);
        auto dh = (float)(newShape.height - new_un_pad[1]);

        if (autoShape)
        {
            dw = (float)((int)dw % stride);
            dh = (float)((int)dh % stride);
        }
        else if (scaleFill)
        {
            dw = 0.0f;
            dh = 0.0f;
            new_un_pad[0] = newShape.width;
            new_un_pad[1] = newShape.height;
            ratio[0] = (float)newShape.width / (float)shape.width;
            ratio[1] = (float)newShape.height / (float)shape.height;
        }

        dw /= 2.0f;
        dh /= 2.0f;

        if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1])
        {
            cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
        }
        else {
            outImage = image.clone();
        }

        int top = int(std::round(dh - 0.1f));
        int bottom = int(std::round(dh + 0.1f));
        int left = int(std::round(dw - 0.1f));
        int right = int(std::round(dw + 0.1f));
        params[0] = ratio[0];
        params[1] = ratio[1];
        params[2] = left;
        params[3] = top;
        cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
    }

    // void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams) {
    //     //cout << maskProtos.size << endl;
    //     int net_width = maskParams.netWidth;
    //     int net_height = maskParams.netHeight;
    //     int seg_channels = maskProtos.size[1];
    //     int seg_height = maskProtos.size[2];
    //     int seg_width = maskProtos.size[3];
    //     float mask_threshold = maskParams.maskThreshold;
    //     Vec4f params = maskParams.params;
    //     Size src_img_shape = maskParams.srcImgShape;
    //     Mat protos = maskProtos.reshape(0, { seg_channels,seg_width * seg_height });
    //     Mat matmul_res = (maskProposals * protos).t();
    //     Mat masks = matmul_res.reshape(output.size(), { seg_width,seg_height });
    //     vector<Mat> maskChannels;
    //     split(masks, maskChannels);
    //     for (int i = 0; i < output.size(); ++i) {
    //         Mat dest, mask;
    //         //sigmoid
    //         cv::exp(-maskChannels[i], dest);
    //         dest = 1.0 / (1.0 + dest);
    //         Rect roi(int(params[2] / net_width * seg_width), int(params[3] / net_height * seg_height), int(seg_width - params[2] / 2), int(seg_height - params[3] / 2));
    //         dest = dest(roi);
    //         resize(dest, mask, src_img_shape, INTER_NEAREST);
    //         //crop
    //         Rect temp_rect = output[i].box;
    //         mask = mask(temp_rect) > mask_threshold;
    //         output[i].boxMask = mask;
    //     }
    // }

void GetMask2(const cv::Mat& maskProposals, const cv::Mat& maskProtos, OutputParams& output, const MaskParams& maskParams) {
	int net_width = maskParams.netWidth;
	int net_height = maskParams.netHeight;
	int seg_channels = maskProtos.size[1];
	int seg_height = maskProtos.size[2];
	int seg_width = maskProtos.size[3];
	float mask_threshold = maskParams.maskThreshold;
	cv::Vec4f params = maskParams.params;
	cv::Size src_img_shape = maskParams.srcImgShape;

	cv::Rect temp_rect = output.box;
	//crop from mask_protos
	int rang_x = floor((temp_rect.x * params[0] + params[2]) / net_width * seg_width);
	int rang_y = floor((temp_rect.y * params[1] + params[3]) / net_height * seg_height);
	int rang_w = ceil(((temp_rect.x + temp_rect.width) * params[0] + params[2]) / net_width * seg_width) - rang_x;
	int rang_h = ceil(((temp_rect.y + temp_rect.height) * params[1] + params[3]) / net_height * seg_height) - rang_y;

	//如果下面的 mask_protos(roi_rangs).clone()位置报错，说明你的output.box数据不对，或者矩形框就1个像素的，开启下面的注释部分防止报错。
	rang_w = MAX(rang_w, 1);
	rang_h = MAX(rang_h, 1);
	if (rang_x + rang_w > seg_width) {
		if (seg_width - rang_x > 0)
			rang_w = seg_width - rang_x;
		else
			rang_x -= 1;
	}
	if (rang_y + rang_h > seg_height) {
		if (seg_height - rang_y > 0)
			rang_h = seg_height - rang_y;
		else
			rang_y -= 1;
	}

	std::vector<cv::Range> roi_rangs;
	roi_rangs.push_back(cv::Range(0, 1));
	roi_rangs.push_back(cv::Range::all());
	roi_rangs.push_back(cv::Range(rang_y, rang_h + rang_y));
	roi_rangs.push_back(cv::Range(rang_x, rang_w + rang_x));

	//crop
	cv::Mat temp_mask_protos = maskProtos(roi_rangs).clone();
	cv::Mat protos = temp_mask_protos.reshape(0, { seg_channels,rang_w * rang_h });
	cv::Mat matmul_res = (maskProposals * protos).t();
	cv::Mat masks_feature = matmul_res.reshape(1, { rang_h,rang_w });
	cv::Mat dest, mask;

	//sigmoid
	cv::exp(-masks_feature, dest);
	dest = 1.0 / (1.0 + dest);

	int left = floor((net_width / seg_width * rang_x - params[2]) / params[0]);
	int top = floor((net_height / seg_height * rang_y - params[3]) / params[1]);
	int width = ceil(net_width / seg_width * rang_w / params[0]);
	int height = ceil(net_height / seg_height * rang_h / params[1]);

	resize(dest, mask, cv::Size(width, height), cv::INTER_NEAREST);
	cv::Rect mask_rect = temp_rect - cv::Point(left, top);
	mask_rect &= cv::Rect(0, 0, width, height);
	mask = mask(mask_rect) > mask_threshold;
	if (mask.rows != temp_rect.height || mask.cols != temp_rect.width) { //https://github.com/UNeedCryDear/yolov8-opencv-onnxruntime-cpp/pull/30
		resize(mask, mask, temp_rect.size(), cv::INTER_NEAREST);
	}
	output.boxMask = mask;

}
    // void DrawPred(Mat& img, vector<OutputSeg> result, std::vector<std::string> classNames, vector<Scalar> color, bool isVideo) {
    //     Mat mask = img.clone();
    //     for (int i = 0; i < result.size(); i++) {
    //         int left, top;
    //         left = result[i].box.x;
    //         top = result[i].box.y;
    //         int color_num = i;
    //         rectangle(img, result[i].box, color[result[i].id], 2, 8);
    //         if (result[i].boxMask.rows && result[i].boxMask.cols > 0)
    //             mask(result[i].box).setTo(color[result[i].id], result[i].boxMask);
    //         string label = classNames[result[i].id] + ":" + to_string(result[i].confidence);
    //         int baseLine;
    //         Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    //         top = max(top, labelSize.height);
    //         //rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
    //         putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 1, color[result[i].id], 2);
    //     }
    //     addWeighted(img, 0.5, mask, 0.5, 0, img); //add mask to src
    //     imshow("1", img);
    //     if (!isVideo )
    //         waitKey(); //video waiKey not in here
    // }

        std::optional<std::vector<OutputParams>> Yolov8Seg::Detect(cv::Mat srcImg){

            auto output = std::vector<OutputParams>();

            cv::Mat blob;
            output.clear();
            int col = srcImg.cols;
            int row = srcImg.rows;
            cv::Mat netInputImg;
            cv::Vec4d params;

            // cv::resize(srcImg, srcImg, cv::Size(640, 640), cv::INTER_LINEAR);
            LetterBox(srcImg, netInputImg, params, cv::Size(_netWidth, _netHeight));

            cv::dnn::blobFromImage(netInputImg, blob  ,1.0 / 255, cv::Size(_netWidth, _netHeight), cv::Scalar(0, 0, 0), true, false);
            
            model.setInput(blob);
            std::vector<cv::Mat> net_output_img;

            std::vector<std::string> output_layer_names{ "output0","output1" };
            model.forward(net_output_img, output_layer_names);
            // model.forward(net_output_img, model.getUnconnectedOutLayersNames());

            std::vector<int> class_ids;// res-class_id
            std::vector<float> confidences;// res-conf 
            std::vector<cv::Rect> boxes;// res-box
            std::vector<std::vector<float>> picked_proposals;  //output0[:,:, 4 + _className.size():net_width]===> for mask
            cv::Mat output0 = cv::Mat(cv::Size(net_output_img[0].size[2], net_output_img[0].size[1]), CV_32F, (float*)net_output_img[0].data).t();  //[bs,116,8400]=>[bs,8400,116]
            int rows = output0.rows;
            int net_width = output0.cols;
            int socre_array_length = net_width - 4 - net_output_img[1].size[1];
            float* pdata = (float*)output0.data;

            for (int r = 0; r < rows; ++r) {
                cv::Mat scores(1, socre_array_length, CV_32FC1, pdata + 4);
                cv::Point classIdPoint;
                double max_class_socre;
                minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
                max_class_socre = (float)max_class_socre;
                if (max_class_socre >= _classThreshold) {
                    std::vector<float> temp_proto(pdata + 4 + socre_array_length, pdata + net_width);
                    picked_proposals.push_back(temp_proto);
                    //rect [x,y,w,h]
                    float x = (pdata[0] - params[2]) / params[0];
                    float y = (pdata[1] - params[3]) / params[1];
                    float w = pdata[2] / params[0];
                    float h = pdata[3] / params[1];
                    int left = MAX(int(x - 0.5 * w + 0.5), 0);
                    int top = MAX(int(y - 0.5 * h + 0.5), 0);
                    class_ids.push_back(classIdPoint.x);
                    confidences.push_back(max_class_socre);
                    boxes.push_back(cv::Rect(left, top, int(w + 0.5), int(h + 0.5)));
                }
                pdata += net_width;//next line
            }
            //NMS
            std::vector<int> nms_result;
            cv::dnn::NMSBoxes(boxes, confidences, _classThreshold, _nmsThreshold, nms_result);
            std::vector<std::vector<float>> temp_mask_proposals;
            cv::Rect holeImgRect(0, 0, srcImg.cols, srcImg.rows);
            for (int i = 0; i < nms_result.size(); ++i) {

                int idx = nms_result[i];
                OutputParams result;
                result.id = class_ids[idx];
                result.confidence = confidences[idx];
                result.box = boxes[idx] & holeImgRect;
                temp_mask_proposals.push_back(picked_proposals[idx]);
                output.push_back(result);
            }
            MaskParams mask_params;
            mask_params.params = params;
            mask_params.srcImgShape = srcImg.size();
            mask_params.netHeight = _netHeight;
            mask_params.netWidth = _netWidth;
            mask_params.maskThreshold = _maskThreshold;
            for (int i = 0; i < temp_mask_proposals.size(); ++i) {
                GetMask2(cv::Mat(temp_mask_proposals[i]).t(), net_output_img[1], output[i], mask_params);
            }

            return output;
        }

        // std::optional<std::vector<OutputParams>> Yolov8Seg::Detect(std::vector<cv::Mat> srcImgs){

        // }

}