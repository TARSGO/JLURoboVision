#include "../Armor.h"
#include "Armor.h"
#include "General.h"
#include "PredictPitch.h"
#include "kalman_filter.hpp"
#include "Thirdparty/angles.h"
#include "Debug/DbgGraph.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "imgui.h"
#include "imgui/imgui_internal.h"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;

static constexpr double
    MaxTrackingDistance = 0.8,
    TrackingThreshold = 5.0,
    LostThreshold = 5.0;

static void CustomTextOntoDrawList(ImDrawList* list, ImVec2 pos, ImU32 col, const char *fmt, ...) {
    const char* text, *text_end;
    va_list args;
    va_start(args, fmt);
    ImFormatStringToTempBufferV(&text, &text_end, fmt, args);
    list->AddText(pos, col, text, text_end);
    va_end(args);
}

void TrackState::AutoAimEKFInit()
{
  //f
  auto f = [](const Eigen::Matrix<double,9,1> & x, Eigen::Matrix<double,1,1> & u)
  {
    Eigen::Matrix<double,9,1> x_pre = x;
    x_pre[0] += x[5] *u[0];
    x_pre[1] += x[6] *u[0];
    x_pre[2] += x[7] *u[0];
    x_pre[3] += x[8] *u[0];
    return x_pre;
  };

  //h
  auto h = [](const Eigen::Matrix<double,9,1> & x)
  {
    Eigen::Matrix<double,4,1> z;
    z(0) = x(0) + x(4) * cos(x(3));
    z(1) = x(1) + x(4) * sin(x(3));
    z(2) = x(2);
    z(3) = x(3);
    return z;
  };

  //Q--由于我们采用恒速模型，故假定加速度=0且满足正态分布，故噪声主要来自于旋转中心平移加速度和旋转加速度，其对位移和速度的影响分别为1/2*dt^2*error和dt*error,故需要引入控制变量u
  auto Q = [](const Eigen::Matrix<double,1,1> & u)
  {
    constexpr double Q_xyz = 20;
    constexpr double Q_yaw = 100;
    constexpr double Q_r   = 800;//FIXME:ekf参数均来自RV，待测试
    Eigen::Matrix<double,9,9> q;
    double dt = u[0];
    double Exyz = 1/2*pow(dt,2)*Q_xyz;
    double Eyaw = 1/2*pow(dt,2)*Q_yaw;
    double Er = 1/2*pow(dt,2)*Q_r;
    double Ev_xyz = dt * Q_xyz;
    double Ev_yaw = dt * Q_yaw;
    q << Exyz*Exyz, 0, 0, 0, 0, Exyz*Ev_xyz, 0, 0, 0,
         0,Exyz*Exyz, 0, 0, 0, 0, Exyz*Ev_xyz, 0, 0,
         0, 0, Exyz*Exyz, 0, 0, 0, 0, Exyz*Ev_xyz, 0,
         0, 0, 0, Eyaw*Eyaw, 0, 0, 0, 0, Eyaw*Ev_yaw,
         0, 0, 0, 0, Er*Er, 0, 0, 0, 0,         
         Ev_xyz*Exyz, 0, 0, 0, 0, Ev_xyz*Ev_xyz, 0, 0, 0, 
         0, Ev_xyz*Exyz, 0, 0, 0, 0, Ev_xyz*Ev_xyz, 0, 0, 
         0, 0, Ev_xyz*Exyz, 0, 0, 0, 0, Ev_xyz*Ev_xyz, 0, 
         0, 0, 0, Ev_yaw*Eyaw, 0, 0, 0, 0, Ev_yaw*Ev_yaw;
    return q;
  };
  //R
  auto R = []()
  {
    constexpr double R_xyz = 0.05;
    constexpr double R_yaw = 0.02;
    Eigen::Matrix<double,4,4> r;
    r << R_xyz, 0, 0, 0,
         0, R_xyz, 0, 0,
         0, 0, R_xyz, 0,
         0, 0, 0, R_yaw;
    return r;
  };
  //P
  Eigen::Matrix<double,9,9> P;
  P.setIdentity();
  //X_post
  Eigen::Matrix<double,9,1> X_post;
  X_post.Zero();
  
  m_ekf=EKF<double,9,4,1>(f,h,Q,R,P,X_post);
}

TrackState::TrackState() :
        m_Kf({}),
        last_jump_position_({ 0, 0, 0 }),
        m_TargetState(Eigen::VectorXd(6).setZero())  
{
//    YAML::Node params = YAML::LoadFile("../General/config.yaml");
    cv::FileStorage file_settings( "../General/config.yaml", cv::FileStorage::READ);
    if (!file_settings.isOpened())
    {
    std::cerr << "Failed to open YAML file in TrackState.cpp" << std::endl;
    return;
    }
    file_settings["m_MaxTrackingDistance"]>>m_MaxTrackingDistance;
    file_settings["m_TrackingThreshold"]>>m_TrackingThreshold;
    file_settings["m_LostThreshold"]>>m_LostThreshold;
    file_settings["m_DetectCount_"]>>m_DetectCount_;
    file_settings["m_FrameCounter"]>>m_FrameCounter;
    file_settings["m_max_match_distance_"]>>m_max_match_distance_;
    file_settings["allow_following_range"]>>allow_following_range;
    file_settings["max_jump_angle"]>>max_jump_angle;
    file_settings["max_jump_period"]>>max_jump_period;
    file_settings["jump_period_"]>>jump_period_;
    file_settings["jump_count_"]>>jump_count_;
    file_settings["accutime"]>>accutime;
    file_settings["spintime"]>>spintime;
    file_settings.release();

    AutoAimEKFInit();
    m_TrackingState= ShootState::LOST;
    KFStateReset();
}

TrackState::~TrackState() {

}

void TrackState::KFStateReset(Eigen::Vector3d initialPosVec) {
    Eigen::Matrix<double, 6, 6> f;
    f <<    1,  0,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,
            0,  0,  1,  0,  0,  0,
            0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  1,  0,
            0,  0,  0,  0,  0,  1;

    // H - measurement matrix
    Eigen::Matrix<double, 3, 6> h;
    h.setIdentity();

    // Q - process noise covariance matrix
    Eigen::DiagonalMatrix<double, 6> q;
    q.diagonal() << 1, 1, 1, 1, 1, 1;

    // R - measurement noise covariance matrix
    constexpr double rconst = 0.05;
    Eigen::DiagonalMatrix<double, 3> r;
    r.diagonal() << rconst, rconst, rconst;

    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 6> p;
    p.setIdentity();

    // Xpost
    Eigen::VectorXd Xpost(6);
    Xpost << initialPosVec, 0, 0, 0;

    m_Kf = KalmanFilter({f, h, q, r, p, Xpost});

}

void TrackState::EKFStateReset(Eigen::Matrix<double,5,1> initialPosVec) {
    //P
    Eigen::Matrix<double,9,9> P;
    P.setIdentity();
    //X_post
    Eigen::Matrix<double,9,1> X_post;
    X_post.Zero();
    X_post << 0, 0, 0, 0, initialPosVec;

    m_ekf.ResetEKF(P,X_post);

}

void TrackState::SetInitialArmor(ArmorDetector& detector)
{
    if (!detector.isFoundArmor())
        return;

    // TODO(chenjun): need more judgement
    // Simply choose the armor that is closest to image center
    double minDistance = DBL_MAX;
    auto& chosenArmor = detector.armors[0];
    auto imgCenter = detector.getImageSize() / 2;
    for (const auto & armor : detector.armors) {
        double distanceToImageCenter = norm((armor.center - cv::Point2f(imgCenter)));
        if (distanceToImageCenter < minDistance) {
            minDistance = distanceToImageCenter;
            chosenArmor = armor;
        }
    }

    // KF SetInitialArmor
    KFStateReset();
    tracking_id = chosenArmor.armorNum;
    m_TrackingState = ShootState::STABILIZE;
}

// 返回值：能否返回给电控Yaw Pitch（即当前跟踪数据能否认为有效）
bool TrackState::UpdateState(ArmorDetector& detector) {
    bool trackingValid = false;
    auto currentTime = high_resolution_clock::now();
    auto timeDiff = duration_cast<microseconds>(currentTime - m_PrevTime);
    double T = timeDiff.count() / 1000000.0;
    accutime+=T;
    // 是否有新帧。时停时用到的一个状态，没有新帧的时候很多东西是不应该更新的
    bool hasNewFrame = false;
    hasNewFrame = FrameFetched.exchange(hasNewFrame);
    static bool enableDistanceBreakpoint = false;
    bool shouldUpdate = true;

    Debug_DisplayStateText();
    bool doUpdateInPause = Debug_SingleFrameOperations();
    shouldUpdate = hasNewFrame || doUpdateInPause;

    // 如果还没有选中初始跟踪的装甲板，先选择，然后返回非法状态
    if(m_TrackingState == ShootState::LOST) {
        SetInitialArmor(detector);
        return false;
    }

    RelCoordAtt pos { 0, 0, 0 };
    ImGui::Begin("Track State");
    if(detector.isFoundArmor()) {
        ArmorBox matched_armor;
        int TrackingArmorIndex = -1;
        double min_position_diff = DBL_MAX;
        int itemIndex = 0;

        m_TargetState = shouldUpdate ? m_Kf.predict(T) : m_Kf.static_predict();

        m_dbgScene.SetPredictedPosition(m_TargetState);
        Eigen::Vector3d predictedPos = m_TargetState.head(3);
        ImGui::Checkbox("Enable this breakpoint", &enableDistanceBreakpoint);
        for (auto & armor : detector.armors) {
            // 判断预测位置与实际位置的差值，小于阈值，则认为追踪依然有效
            Eigen::Vector3d resolvedPos(armor.resolvedPos.x, armor.resolvedPos.y, armor.resolvedPos.z);
            //cout << "/ ==== resolvedPos\n" << resolvedPos << "\n\\ ==== resolvedPos" << endl;
            double distance = ((predictedPos - resolvedPos)).norm();
            armor.distanceDiff = distance;
            //if(hasNewFrame && enableDistanceBreakpoint);
            if(distance <= min_position_diff ) {
                min_position_diff = distance;
                matched_armor = armor;
                TrackingArmorIndex = itemIndex;
                matchedarmornum = matched_armor.armorNum;
            }
            else matchedarmornum=0;
            // 在可视化UI里显示该目标位置
            // 交换Y和Z轴。可视化用的ImGuizmo的坐标不太一样
            // FIXME:轴不对，先注释掉，后面测试。
//            std::swap(armor.rMat.at<double>(0, 1), armor.rMat.at<double>(0, 2));
//            std::swap(armor.tMat.at<double>(0, 1), armor.tMat.at<double>(0, 2));
            // 坐标变换。AngleSolver里得到的是1x3的旋转向量，需要先转换成3x3的旋转矩阵
            cv::Rodrigues(armor.rMat, armor.rMat);
            m_dbgScene.SetGizmoAttitude(itemIndex++, armor.rMat, armor.tMat);
        }
        if(detector.isFoundArmor())
            Debug_ArmorPnPResultGraph(detector.armors[0], shouldUpdate);

        Debug_ArmorOnScreenPos(detector, TrackingArmorIndex, predictedPos);
        m_dbgScene.Frame(itemIndex);


        Eigen::Vector3d resolvedPos(matched_armor.resolvedPos.x,
                                    matched_armor.resolvedPos.y,
                                    matched_armor.resolvedPos.z);


        if(m_TrackingState >= ShootState::STABILIZE && shouldUpdate) {
            m_TargetState = m_Kf.update(resolvedPos);
        }
        if (m_TrackingState > ShootState::STABILIZE) {
            // 只有当卡尔曼滤波器已经收敛之后才能开始判断距离
            if (min_position_diff < m_max_match_distance_) {
                // Matching armor found
                trackingValid = true;
                if(shouldUpdate)
                    m_TargetState = m_Kf.update(resolvedPos);
            } else if (m_TrackingState >= ShootState::FOLLOWING) {
                // 正常跟踪情况下才能由于对方旋转、丢失跟踪而进入小陀螺模式
                // 没有任何一块装甲板在阈值内。进入小陀螺检测逻辑，
                // 如果有相同标号的装甲板，则认为它是小陀螺要跟踪的下一个目标
                for (const auto &armor: detector.armors) {
                    if (armor.armorNum == tracking_id) {
                        trackingValid = true;
                        // 硬推一下KF
                        KFStateReset(armor.resolvedPos);
                        Eigen::Vector3d resolvedPosVec = armor.resolvedPos;
                        Eigen::VectorXd newTargetState;
                        newTargetState.setZero(6);
                        newTargetState << resolvedPosVec, m_TargetState[3], m_TargetState[4], m_TargetState[5];
                        m_TargetState = newTargetState;
                        // 判断进入小陀螺模式
                        double current_yaw = atan2(m_TargetState(1), m_TargetState(0));//计算yaw
                        //double current_yaw = std::atan2(current_position.y(), current_position.x());
                        double yaw_diff = get_shortest_angular_distance(last_yaw_, current_yaw);

                        if (std::abs(yaw_diff) > max_jump_angle) {
                            jump_count_++;
                            if (jump_count_ > 1 && std::signbit(yaw_diff) == std::signbit(last_jump_yaw_diff_)) {
                                jump_period_ = spintime;
                                m_TrackingState = ShootState::SPINNING;
                            }
                            auto spintimeDiff = duration_cast<microseconds>(currentTime - last_jump_time_);
                            spintime = spintimeDiff.count() / 1000000.0;
                            last_jump_time_ = currentTime;
                            last_jump_position_ = {m_TargetState(0) ,m_TargetState(1) ,m_TargetState(2)};
                            last_jump_yaw_diff_ = yaw_diff;
                            accutime=0;//if in spinning,clear the accutime
                        }
                        ImGui::Begin("spin00");
                        ImGui::Text("yaw_diff:%lf",yaw_diff);
                        //ImGui::Text("")
                        ImGui::End();
                        last_yaw_ = current_yaw;
                        break;
                    }
                }

            }
        }

        // DEBUG DISPLAY
        if (matched_armor.type == ArmorType::SMALL_ARMOR){
            ImGui::Text("Armor Type: SMALL ARMOR" );
        }
        else if (matched_armor.type == ArmorType::BIG_ARMOR){
            ImGui::Text("Armor Type: BIG ARMOR" );
        }
        ImGui::Text("Matched Armor Number %d", matched_armor.armorNum - '0');
        ImGui::Text("Pos\nX %.3f\nY %.3f\nZ %.3f", pos.x, pos.y, pos.z);

        if(m_TargetState.rows() >= 6) {
            // Only do this when m_TargetState has data
            ImGui::Text("KF Position\nX %.3f\nY %.3f\nZ %.3f\nKF Velocity\nX %.3f\nY %.3f\nZ %.3f",
                        m_TargetState(0),
                        m_TargetState(1),
                        m_TargetState(2),
                        m_TargetState(3),
                        m_TargetState(4),
                        m_TargetState(5));
        }

        {
            static DbgGraph minDiffGraph(100);
            if(shouldUpdate) minDiffGraph.NewData(min_position_diff);
            minDiffGraph.Frame("MinDiff", nullptr, 0.0f, 150.0f, ImVec2(0, 80.0f));
        }
    }else
    {
        matchedarmornum=0;
    }

    // 状态机更新
    if(shouldUpdate) {
        switch(m_TrackingState) {
            case ShootState::LOST:
                break;

            case ShootState::STABILIZE:
                // 等待Kalman收敛而等待的数帧
                if(m_FrameCounter++ >= 4) {
                    m_FrameCounter = 0;
                    if(detector.isFoundArmor())
                        m_TrackingState = ShootState::WAITING;
                    else {
                        m_TrackingState = ShootState::LOST;
                        KFStateReset();
                    }
                }
                break;
                
            case ShootState::WAITING:
                // 在一段时间跟踪后确认当前跟踪的目标，切换到已识别状态
                if(!detector.isFoundArmor() || !trackingValid) {
                    m_TrackingState = ShootState::LOST;
                    KFStateReset();
                    break;
                }
                if(m_FrameCounter++ >= 5) {
                    m_FrameCounter = 0;
                    m_TrackingState = ShootState::FOLLOWING;
                }
                break;

            case ShootState::FOLLOWING:
                // 已经认为获得了稳定的跟踪状态
                if (!trackingValid) {
                    // 丢了？没有完全丢的状态，此时可以重新获得跟踪
                    // 小陀螺应该也是从这个状态进入的
                    m_FrameCounter = 0;
                    m_TimeSecondCounter = currentTime;
                    m_TrackingState = ShootState::LOSING_FOLLOW;
                    doFire = true;
                }
                break;
            case ShootState::LOSING_FOLLOW: {
                // 丢失跟踪状态，等待一段时间后切换到丢失状态。
                // 注意这个状态下依然可以触发小陀螺
                constexpr double LoseTrackingThreshold = 0.3; // 秒
                double lostTime =
                        duration_cast<microseconds>(currentTime - m_TimeSecondCounter).count() / 1000000.0;
                if (!trackingValid) {
                    if (lostTime > LoseTrackingThreshold) {
                        m_TrackingState = ShootState::LOST;
                    }
                    doFire = false;
                } else {
                    m_TrackingState = ShootState::FOLLOWING;
                    doFire = true;
                }
                break;
            }
            case ShootState::SPINNING: {
                if ((abs(spintime - jump_period_) < allow_following_range) && accutime < 5 * spintime) {//the first half:judge each observed time matches the initial measurement;the second half:judge whether the target in spinning
                    doFire = true;
                }
                else {
                    m_TargetState(0) = last_jump_position_.x();
                    m_TargetState(1) = last_jump_position_.y();
                    m_TargetState(2) = last_jump_position_.z();
                    m_TargetState(3) = 0;
                    m_TargetState(4) = 0;
                    m_TargetState(5) = 0;

                    doFire = false;
                    m_TrackingState = ShootState::LOSING_FOLLOW;
                }
            }
            break;
        }
    }


    ImGui::End();
    m_PrevTime = currentTime;


    switch(m_TrackingState) {
        case ShootState::LOST:
        case ShootState::STABILIZE:
        case ShootState::WAITING:
            return false;
        case ShootState::FOLLOWING:
        case ShootState::SPINNING:
            return true;
        default:
            return false;
    }
}


double TrackState::get_shortest_angular_distance(double last_yaw, double current_yaw)
{
    double temp = current_yaw - last_yaw;
    if (temp > M_PI)
        temp -= 2 * M_PI;
    else if (temp < -M_PI)
        temp += 2 * M_PI;
    return temp;
}

void TrackState::Debug_ArmorOnScreenPos(ArmorDetector &detector, int trackingArmorIdx, Eigen::Vector3d predictXyz) {
    // 装甲板（位于）屏幕（上的）位置可视化（调试用）
    static bool doDisplayArmorScreenPos = true;
    ImGui::Begin("Armor On-Screen Position");
    ImGui::Checkbox("Enabled", &doDisplayArmorScreenPos);
    auto pred = Reproject(predictXyz);
    ImGui::Text("Pred pos on Image (%f, %f)", pred.x, pred.y);
    auto imgSize = detector.getImageSize();
    if(doDisplayArmorScreenPos) {
        ImVec2 canvasP0 = ImGui::GetCursorScreenPos(), canvasSize = ImGui::GetContentRegionAvail();
        canvasSize.y = canvasSize.x * ((float)imgSize(1) / imgSize(0)); // 按比例，以可用宽度为基准改变高度
        ImVec2 canvasP1(canvasP0.x + canvasSize.x, canvasP0.y + canvasSize.y);
        ImDrawList *drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(canvasP0, canvasP1, IM_COL32(50, 50, 50, 255));
        drawList->AddRect(canvasP0, canvasP1, IM_COL32_WHITE);

        drawList->PushClipRect(canvasP0, canvasP1, true);
        int armorIndex = 0;
        for(const auto & armor : detector.armors) {
            auto centerImg = armor.center;
            ImVec2 centerGui = canvasP0;
            centerGui.x += centerImg.x * canvasSize.x / imgSize(0);
            centerGui.y += centerImg.y * canvasSize.y / imgSize(1);
            drawList->AddCircle(centerGui, 5, IM_COL32(255, 150, 150, 255), 0, 1.5);
            if(trackingArmorIdx == armorIndex++) {
                drawList->AddCircle(centerGui, 8, IM_COL32(100, 255, 100, 255), 0, 1.5);
            }

            ImVec2 textPos(centerGui); textPos.y += 10; textPos.x -= 20;
            CustomTextOntoDrawList(drawList, textPos, IM_COL32_WHITE, "Dist %.2f", armor.distanceDiff);
        }
        // Draw Prediction point
        ImVec2 predPoint(canvasP0.x + (pred.x / imgSize(0)) * canvasSize.x,
                         canvasP0.y + (pred.y / imgSize(1)) * canvasSize.y);
        drawList->AddLine(ImVec2(predPoint.x - 4, predPoint.y - 4),
                          ImVec2(predPoint.x + 4, predPoint.y + 4),
                          IM_COL32(150, 0, 0, 255),
                          1.5);
        drawList->AddLine(ImVec2(predPoint.x + 4, predPoint.y - 4),
                          ImVec2(predPoint.x - 4, predPoint.y + 4),
                          IM_COL32(150, 0, 0, 255),
                          1.5);

        drawList->PopClipRect();
    }
    ImGui::End();
}

Eigen::VectorXd TrackState::GetTargetState() {
    return m_TargetState;
}

void TrackState::Debug_DisplayStateText() {
    ImGui::Begin("Track State");
    switch(m_TrackingState) {
        case ShootState::LOST:
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "LOST");
            break;
        case ShootState::STABILIZE:
            ImGui::TextColored(ImVec4(1, 0.8, 0, 1), "STABILIZE");
            break;
        case ShootState::WAITING:
            ImGui::TextColored(ImVec4(1, 0.8, 0, 1), "WAITING");
            break;
        case ShootState::FOLLOWING:
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "FOLLOWING");
            break;
        case ShootState::LOSING_FOLLOW:
            ImGui::TextColored(ImVec4(1, 0.8, 0, 1), "LOSING");
            break;
        case ShootState::SPINNING:
            ImGui::TextColored(ImVec4(1, 0.6, 0.6, 1), "SPINNING");
            break;
    }
    ImGui::End();
}

void TrackState::Debug_ArmorPnPResultGraph(ArmorBox &armor, bool newData) {
    constexpr int graphsize = 100;
    static DbgGraph gX(graphsize), gY(graphsize), gZ(graphsize), g1(graphsize), g2(graphsize), g3(graphsize);

    if(newData) {
        gX.NewData(armor.tMat.at<double>(0));
        gY.NewData(armor.tMat.at<double>(1));
        gZ.NewData(armor.tMat.at<double>(2));
        g1.NewData(armor.rMat.at<double>(0));
        g2.NewData(armor.rMat.at<double>(1));
        g3.NewData(armor.rMat.at<double>(2));
    }

    ImGui::Begin("Armor PnP Graph");
    gX.Frame("X", nullptr, -1000, 1000, ImVec2(0, 50));
    gY.Frame("Y", nullptr, -1000, 1000, ImVec2(0, 50));
    gZ.Frame("Z", nullptr, -3000, 3000, ImVec2(0, 50));
    g1.Frame("1", nullptr, -4, 4, ImVec2(0, 50));
    g2.Frame("2", nullptr, -4, 4, ImVec2(0, 50));
    g3.Frame("3", nullptr, -4, 4, ImVec2(0, 50));
    ImGui::End();
}

bool TrackState::Debug_SingleFrameOperations() {
    static bool doSingleFrame = false;
    // 是否在时停时更新状态。一般是不用的，但是万一用得到呢？
    static bool enableUpdateInPause = false;

    ImGui::Begin("Track State");

    ImGui::Checkbox("Single frame", &doSingleFrame); ImGui::SameLine();
    SingleFrameMode.store(doSingleFrame);
    SingleFrameFlag = ImGui::Button("Next frame");
    ImGui::Checkbox("Update states in pause", &enableUpdateInPause);

    ImGui::End();
    return enableUpdateInPause;
}

void TrackState::SetCameraMatrix(cv::Mat& matrix) {
    cv::cv2eigen(matrix, m_CameraMatrix);
}

cv::Point2f TrackState::Reproject(Eigen::Vector3d& xyz) {
    auto result = (1.0 / xyz(2)) * m_CameraMatrix * xyz;
    return cv::Point2f(result(0), result(1));
}


