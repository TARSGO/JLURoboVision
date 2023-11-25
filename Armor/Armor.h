/*
*	@Author: Mountain
*	@Date:	 2020.04.13
*	@Brief:  This header file declares all the classes and params used to detect/recognize enemy armors
*/

#ifndef ARMOR
#define ARMOR

#include "../General/General.h"
#include "../Serial/Serial.h"
#include "Armor/predictor/kalman_filter.hpp"
#include "ImguiDbgkit.h"
#include "Debug/Dbg3DScene.h"
#include "./predictor/EKF.hpp"
#include "time.h"
#include <chrono>
#include <cmath>
#include "Armor/predictor/PredictPitch.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
enum class DetectorState
{
    LIGHTS_NOT_FOUND = 0,
    LIGHTS_FOUND = 1,
    ARMOR_NOT_FOUND = 2,
    ARMOR_FOUND = 3
};

enum class ShootState
{
    LOST,           //
    LOSING_FOLLOW,      //
    STABILIZE,      //
    WAITING,        //    
    FOLLOWING,       //
    SPINNING,
};

class LightBar
{
public:

    LightBar();
/**
 *@brief: Parametrical constructor of lightBar 灯条有参构造函数
 *@param: RotatedRect created by fitellipse  拟合椭圆获得的旋转矩形来构造灯条
 */
    LightBar(const cv::RotatedRect &lightRect,double dbgarea);
    ~LightBar();

public:
    cv::RotatedRect lightRect; //rotation rect of light 灯条的旋转矩形（椭圆拟合获得）
    double length;
    double width;
    float angle;
    Point2f center; //center of light bar 灯条中心
    Point2f top, bottom;
    double area;
};



/**
 * @brief: information of Armor 装甲板相关数据信息
 */
class ArmorBox
{
public:
    ArmorBox();
/**
 *@brief: Parametrical constructor of armorBox 装甲板有参构造函数
 *@param: two LightBar  左右两个灯条
 */
    ArmorBox(const LightBar& l_light, const LightBar& r_light);
    ~ArmorBox();

    //judge if this is the same armor box
    bool operator==(ArmorBox& b);

    // dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
    float getDislocationX() const;

    // an integrative function to judge whether this armor is suitable or not
    bool isSuitableArmor() const;

public:
    LightBar l_light, r_light; //the left and right lightbar of this armor 装甲板的左右灯条
    int l_index, r_index; //the index of left and right light 左右灯条的下标(默认为-1，仅作为ArmorDetector类成员时生效)
    int armorNum;  //number on armor(recognized by SVM) 装甲板上的数字（用SVM识别得到）
    vector<Point2f> armorVertices;  // bl->tl->tr->br     左下 左上 右上 右下
    ArmorType type; //the type of armor
    Point2f center;	// center point(crossPoint) of armor 装甲板中心
    float armorAngle;//armor angle(mean of lightBars) 装甲板角度(灯条角度的平均值)
    Mat armorImg;	//image of armor set by getArmorImg() from ArmorNumClassifier() 装甲板的图片（透射变换获得）

    RelCoordAtt resolvedPos; // 解算位置得出的该装甲板的XYZ坐标
    RotationAtt resolvedAng; // 解算位置得出的该装甲板的欧拉角
    Mat rMat, tMat; // 为ImGuizmo可视化缓存的旋转矩阵和平移矩阵
    double distanceDiff;
    double confidence;
    std::string classfication_result;
};



/**
 * @brief: use warpPerspective to get armorImg 利用透射变换截取装甲板图片
 */
class ArmorNumClassifier
{
public:
    ArmorNumClassifier();
    ~ArmorNumClassifier();

    /**
     * @brief: load the current roiImage from ArmorDetector 载入roiImage（剪切出装甲板）
     * @param: the path of xml_file  待载入SVM模型的路径
     */
    void loadImg(Mat & srcImg);

    /**
     * @brief: use warpPerspective to get armorImg  利用透视变换获得装甲板图片
     * @param: the path of xml_file  待载入SVM模型的路径
     */
    void getArmorImg(ArmorBox& armor);

    /**
     * @brief: use SVM to recognize the number of each Armor 利用SVM实现装甲板数字识别
     */
    void setArmorNum(ArmorBox& armor);

    void showArmorNum();

    double threshold = 0.7;

private:
    cv::dnn::Net net_;
    std::vector<char> class_names_;

    Mat number_image;
    Mat warpPerspective_src; //warpPerspective srcImage  透射变换的原图

    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
};

class ArmorDetector
{
public:
    ArmorDetector();
    ~ArmorDetector();

    /**
     * @brief: set enemyColor  设置敌方颜色
     */
    void setEnemyColor(int  enemyColor);

    /**
     *@brief: reset the ArmorDetector(delete the priviois lights and armors) to start next frame detection 重设检测器（删除原有的灯条和装甲板s）和装甲板状态，以便进行下一帧的检测
     */
    void resetDetector();

    /**
     * @brief: load source image and set roi if roiMode is open and found target in last frame 载入源图像并进行图像预处理
     * @param: const Mat& src     源图像的引用
     */
    void setImg(Mat & src);

    /**
     * @brief: find all the possible lights of armor (get lights) 检测所有可能的灯条
     */
    void findLights();

    /**
    * @brief: match lights into armors (get armors) 将识别到的灯条拟合为装甲板
    */
    void matchArmors();

    /**
    *@brief: erase wrong armor that used a same light with another armor 擦除与某装甲板共用了一根灯条的错误装甲板
    */
    void eraseErrorRepeatArmor();
    
    /**
    *@brief: erase wrong armor that crossed a light擦除跨越灯条匹配出的装甲板
    */
    void eraseCrossLightsArmor();

    /**
     *@brief: an integrative function to run the Detector 集成的装甲板检测识别函数
     */
    void run(Mat & src);

    /**
     *@brief: get the target armor
     */
    ArmorBox getTarget();

    /**
     *@brief: return the Detector status 识别程序是否识别到装甲版
     *@return: FOUND(1) NOT_FOUND(0)
     */
    bool isFoundArmor();
    /**
     *@brief: return the Detector status 识别程序是否识别到装甲版
     *@return: LOST(0) FOLLOWING(1) FOLLOWING(2) SPINNING(3) ARMOR_SWITCH(4) CAR_SWITCH(5)
     */
    bool isLight(const LightBar & light);

    bool ArmorJudge(ArmorBox armor);


    /**
     * @brief 获取传入图像的分辨率
     * @return cv::Vec2i { Xres, Yres }
     */
    cv::Vec2i getImageSize() { return { srcImg.cols, srcImg.rows }; }

    /**
     *@brief: show all the informations of this frame detection  显示所有信息
     */
    void showDebugInfo(bool showSrcImg_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON);

    void SetCameraMatrix(cv::Mat&);

    cv::Point2f Reproject(Eigen::Vector3f xyz);

    void showArmors(Mat & image, const vector<ArmorBox> & armors, const vector<LightBar>& lights,Eigen::Vector3f absxyz_show,Eigen::Vector3f camxyz_show);


    bool containLight(const LightBar & light_1, const LightBar & light_2, const std::vector<LightBar> & lights);

    vector<ArmorBox> armors; //all the armors matched from lights 识别到的所有装甲板
    std::vector<ArmorBox>::iterator begin() { return armors.begin(); }
    std::vector<ArmorBox>::iterator end() { return armors.end(); }

    Eigen::Vector3f absxyz_show;
    Eigen::Vector3f camxyz_show;
    Eigen::Vector3f predxyz_show;
private:
    int thresh = 240;//240
    Mat srcImg;  //source image (current frame acquired from camera) 从相机采集的当前的图像帧
    Mat srcImg_binary; //binary image of srcImg 源图像的二值图
    Mat src_roi;
    Mat src_full;
    Color enemyColor;  //the color of enemy 敌方颜色
    Mat grayImg;
    vector<LightBar> lights; //all the lightBars find in roiIng 找到的灯条
    ArmorBox targetArmor; //current target for current frame 当前图像帧对应的目标装甲板
    ArmorBox lastArmor;  //previous target for last frame 上一帧图像的目标装甲板
    ArmorNumClassifier classifier; //class used to get armorImg and classifier the armorNum 获取装甲板图像及识别装甲板数字的类
    DetectorState state; //the state of detector updating along with the program running 装甲板检测器的状态，随着装甲板进程的执行而不断更新
    // 相机参数，重投影用
    Eigen::Matrix3f m_CameraMatrix;


};


class Outpost 
{
public:
  void AntiOutpostEKFInit();
  //Eigen::VectorXd FixOutpost(Eigen::Matrix <double,4,1> m_ArmorState, double dt);//使用恒角速度模型对前哨站进行预测和解算
private:  
  EKF<double,9,4,1> CV_EKF;//装载恒角速度模型
};


    // 维护跟踪装甲板、解算目标车辆的状态
class TrackState {
public:
    TrackState();
    ~TrackState();

    void SetCameraMatrix(cv::Mat&);

    void SceneNewFrame() { m_dbgScene.BeginFrame(); }
    bool UpdateState(ArmorDetector &detector);

    double get_shortest_angular_distance(double last_yaw,double current_yaw);
    Eigen::VectorXd GetTargetState();

    int tracking_id;
    Eigen::VectorXd m_TargetState;
    bool doFire;


protected:
    void AntiSpinInit();
    void AutoAimEKFInit();
    //反前哨站先用EKF去迭代得到大致的速度及速度方向，根据该信息确定恒速模型的速度取值
    void AntiOutpostEKFInit(Eigen::VectorXd targetstate);
    void SetInitialArmor(ArmorDetector &detector);
    void KFStateReset(Eigen::Vector3d initialPosVec = Eigen::Vector3d().setZero());
    void EKFStateReset(Eigen::Matrix<double,9,1> initialPosVec = Eigen::Matrix<double,9,1>().setZero());
    ArmorBox ChooseArmor(ArmorDetector &detector);
    void BackToFollow();
    Eigen::VectorXd GetArmorState(const ArmorBox & target);
    bool RespondState(const ShootState & track_state);//针对状态作应答
    void JudgeState();//判断状态
    cv::Point2f Reproject(Eigen::Vector3d& xyz);

    void Debug_ArmorOnScreenPos(ArmorDetector &detector, int trackingArmorIdx, Eigen::Vector3d predictXyz);
    void Debug_DisplayStateText();
    void Debug_ArmorPnPResultGraph(ArmorBox &armor, bool newData);
    bool Debug_SingleFrameOperations();

private:

    double m_MaxTrackingDistance, m_TrackingThreshold, m_LostThreshold;

    KalmanFilter m_Kf;
    EKF<double,9,4,1> m_ekf;


    ShootState m_TrackingState;// 维护追踪器的状态
    Eigen::Matrix <double,4,1> m_ArmorState;//装甲板运动序列
    bool isFoundTarget;
    bool isTrackVaild;
    bool isAntiOutpost;
    int m_DetectCount_;
    int m_LostCount_;
    int m_FrameCounter;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_TimeSecondCounter; // 计秒变量，储存计时开始时间
    std::chrono::time_point<std::chrono::high_resolution_clock> m_PrevTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_jump_time_;
    // 相机参数，重投影用
    Eigen::Matrix3d m_CameraMatrix;

    Dbg3DScene m_dbgScene;
    double min_position_diff;
    Eigen::Vector3d m_tracking_velocity_;

    double m_max_match_distance_ ;
    double allow_following_range;
    double max_jump_angle;
    double max_jump_period;

    bool target_spinning_;
    double jump_period_;//即当前的观测旋转周期T
    int jump_count_;

    double spintime;// time of each observe period
    double accutime;//time of each frame(it will be cleaned if target in spinning)
    double last_yaw_;
    double last_jump_yaw_diff_;

};

#endif // !ARMOR
