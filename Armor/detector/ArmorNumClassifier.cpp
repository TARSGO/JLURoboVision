/*
*	@Author: Chenjun
*	@Date:	 忘了
*	@Brief:  This cpp file define the ArmorNumClassifier class, realize some function used 
*/
#include "Armor/Armor.h"
#include <iostream>
#include <fstream>
#include <random>

using namespace std;
using namespace cv;

ArmorNumClassifier::ArmorNumClassifier()
{
    net_ = cv::dnn::readNetFromONNX("../General/mlp.onnx");
    std::ifstream label_file("../General/label.txt");
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line[0]);
    }
}

ArmorNumClassifier::~ArmorNumClassifier() {}

void ArmorNumClassifier::loadImg(Mat & srcImg){
	//copy srcImg as warpPerspective_src
	(srcImg).copyTo(warpPerspective_src);
	//preprocess srcImg for the goal of acceleration
}

void ArmorNumClassifier::getArmorImg(ArmorBox & armor)
{
    // Number ROI size
    const cv::Size roi_size(20, 28);

    cv::Point2f lights_vertices[4] = {
            armor.l_light.bottom, armor.l_light.top, armor.r_light.top,
            armor.r_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL_ARMOR ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(0, top_light_y),
            cv::Point(warp_width - 1, top_light_y),
            cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_img;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_img, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    number_img =
            number_img(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_img, number_img, cv::COLOR_RGB2GRAY);
    cv::threshold(number_img, number_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    number_image = number_img;
}

void ArmorNumClassifier::setArmorNum(ArmorBox & armor){
    cv::Mat image = number_image.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.armorNum = class_names_[label_id];

    std::stringstream result_ss;
    result_ss << class_names_[label_id] - '0' << ":_" << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
    cout<<armor.armorNum<<endl;
}

//如果需要制作数据集，改变输出文件夹并取消imwrite的注释即可
void ArmorNumClassifier::showArmorNum()
{
    // 生成随机文件名
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, 999999);

    String output_folder = "/home/zhouhb/桌面/6/";  // 指定输出文件夹
    //imwrite(output_folder + to_string(dis(gen)) + ".png", number_image);  // 将图像保存为PNG格式
    imshow("MLP" ,number_image);
}
