#include "Armor/Armor.h"


ArmorDetector::ArmorDetector()
{
    state = DetectorState::LIGHTS_NOT_FOUND;
}

ArmorDetector::~ArmorDetector(){}

void ArmorDetector::resetDetector()
{
    state = DetectorState::LIGHTS_NOT_FOUND;
    lights.clear();
    armors.clear();
}

/**
* @brief: set enemyColor  设置敌方颜色
*/
void ArmorDetector::setEnemyColor(int enemyColor)
{
    if(enemyColor == 1)
    {
        this->enemyColor = Color::BLUE;
    }
    else if (enemyColor == 0)
    {
        this->enemyColor = Color::RED;
    }
}

/**
* @brief: load source image and set roi if roiMode is open and found target in last frame 载入源图像并设置ROI区域（当ROI模式开启，且上一帧找到目标装甲板时）
* @param: const Mat& src     源图像的引用
*/
void ArmorDetector::setImg(Mat & src){
    src.copyTo(src_full);  //deep copy src to srcImg 深（值）拷贝给srcImg
    classifier.loadImg(src_full); //srcImg for classifier, warp perspective  载入classifier类成员的srcImg，用于透射变换剪切出装甲板图
    src_full.copyTo(srcImg);  //deep copy src to srcImg 深（值）拷贝给srcImg

    cv::cvtColor(srcImg, grayImg, cv::COLOR_RGB2GRAY);
    cv::threshold(grayImg, srcImg_binary, thresh, 255, cv::THRESH_BINARY);

}


/**
 *@brief: an integrative function to run the Detector 集成跑ArmorDetector
 */
void ArmorDetector::run(Mat & src) {
    //firstly, load and set srcImg  首先，载入并处理图像
    setImg(src); //globally srcImg and preprocess it into srcImg_binary 载入Detector的全局源图像 并对源图像预处理成

    //secondly, reset detector before we findLights or matchArmors(clear lights and armors we found in the last frame and reset the state as LIGHTS_NOT_FOUND)
    //随后，重设detector的内容，清空在上一帧中找到的灯条和装甲板，同时检测器状态重置为LIGHTS_NOT_FOUND（最低状态）
    resetDetector();

    //thirdly, find all the lights in the current frame (srcImg)
    //第三步，在当前图像中找出所有的灯条
    findLights();

    //forthly, if the state is LIGHTS_FOUND (detector found more than two lights) , we match each two lights into an armor
    //第四步，如果状态为LIGHTS_FOUND（找到多于两个灯条），则
    if (state == DetectorState::LIGHTS_FOUND)
    {
        //match each two lights into an armor and if the armor is a suitable one, emplace back it into armors
        //将每两个灯条匹配为一个装甲板，如果匹配出来的装甲板是合适的，则压入armors中
        matchArmors();
    }
}

ArmorBox ArmorDetector::getTarget()
{
    return targetArmor;
}

/**
 *@brief: return the Detector status 识别程序是否识别到装甲版
 *@return: FOUND(1) NOT_FOUND(0)
 */
bool ArmorDetector::isFoundArmor()
{
    return armors.size() > 0 && armors.size() <= 7;
}

