#include <fstream>
#include "../General/General.h"
#include "../Armor/Armor.h"
#include "../AngleSolver/AngleSolver.h"
#include "General.h"
#include "ImguiDbgkit.h"
#include "Dbg3DScene.h"
#include "Armor/predictor/PredictPitch.h"
#include <numeric>

// #define takePhoto
#define DEBUG_MODE
//FIXME：这个模式切换，回头会改成以测距远近为准
//#define AIM_OR_LASER 1 //AIM:1 LASER:2 


ArmorDetector Armor;
AngleSolver angleSolver;
TrackState trackState;
PredictPitch bullet;
SerialReceiveData received;

double TxPrevYaw = 0.0, TxPrevPitch = 0.0, TxPrevDist = 0.0;
int TxPrevArmorNum = 0;

void armorDetectingThread()
{
    //Set angle solver prop
    angleSolver.setCameraParam();
    angleSolver.setArmorSize(ArmorType::SMALL_ARMOR, 75, 67);
    angleSolver.setArmorSize(ArmorType::BIG_ARMOR,225,110);

    auto camMat = angleSolver.getCameraMatrix();
    Armor.SetCameraMatrix(camMat);
    trackState.SetCameraMatrix(camMat);

    this_thread::sleep_for(chrono::milliseconds(1000));

    double t;
    int frameCount = 0;

    while (!ImguiDbgkit::get()->IsDone()) {
        ImguiDbgkit::get()->NewFrame();
        trackState.SceneNewFrame();

        // FPS
        t = cv::getTickCount();

        // Receive from serial port
        static char SerialDeviceName[256] = "/dev/ttyACM0";
        ImGui::Begin("Serial Port Control");
        ImGui::InputText("Serial Port Device", SerialDeviceName, sizeof(SerialDeviceName));
        ImGui::End();
        Serial::Get()->SerialReceive(&received, SerialDeviceName);
        ImGui::Begin("Received");
        ImGui::Text("yaw:%lf",received.yaw);
        ImGui::Text("Pitch:%lf",received.pitch);
        ImGui::End();
        //consumer gets image
        if (1) {
            unique_lock<mutex> lck(Globalmutex);
            //imageReadable = true;
            while (!imageReadable) {
                GlobalCondCV.wait(lck);
            }
            imageReadable = false;
        }

      if (received.mode == 1)
      {
          //Armor.setImg(src);
          //装甲板检测识别子核心集成函数
          Armor.run(src);
          auto Cbn = GetRotMatq4(received.INS_quat1, received.INS_quat2, received.INS_quat3, received.INS_quat4);
#ifdef takePhoto
          if(frameCount == 10)
          {

          string location = "./photo/";
          string name = to_string(t);
          string end = ".jpg";
          string finalName = location + name;
          finalName += end;
          cv::imwrite(finalName,src);
          frameCount = 0;
          }
          frameCount += 1;
#endif
          //Set armor detector prop
          //Armor.setEnemyColor(received.is_enemy_blue); //here set enemy color
          //FIXME:这个似乎是反的...
          Armor.setEnemyColor(1); 
          
          //迭代所有检测到的装甲板，解算出他们当前的位置，便于tracker处理
          for(auto i = Armor.begin(); i != Armor.end(); i++) {
              auto Rc = angleSolver.getArmorPos(*i);
              //FIXME: 当前使用相机坐标系坐标，记得改回来
              //auto Rn = Cbn * Rc;
              auto Rn = Rc;
              Armor.absxyz_show = Eigen::Vector3f(Rc);

              ImGui::Begin("Tracker");
              ImGui::Text("X: %lf", Rc(0) );
              ImGui::Text("Y: %lf", Rc(1) );
              ImGui::Text("Z: %lf", Rc(2) );
              ImGui::Text("absX: %lf", Rn(0) );
              ImGui::Text("absY: %lf", Rn(1) );
              ImGui::Text("absZ: %lf", Rn(2) );
              ImGui::End();

              i->resolvedPos = { Rn(0), Rn(1), Rn(2) };
          }
          if(received.bullet_speed == 0)received.bullet_speed = 15.7;
          bool targetValid = trackState.UpdateState(Armor);
          RotationAtt rotAtt, rotAtt1;
          if(targetValid) {
              Eigen::VectorXd target;
              target = trackState.GetTargetState();
              float T = 0.2f + 0.5f; //T = 机械（拨弹）延迟 + 电控延迟（ms级） + 视觉延迟 + 串口延迟（ms级） + bullet  time
              float preX, preY, preZ;
              preX = target(0) + target(3) * T;
              preY = target(1) + target(4) * T;
              preZ = target(2) + target(5) * T;
              rotAtt1 = xyz2PitchYawDis(preX, preY, preZ);

              //FIXME:记得改回惯性坐标系
              // auto cam = Cbn.inverse() * Eigen::Vector3f{preX ,preY, preZ};
              auto cam=Eigen::Vector3f{preX ,preY, preZ};
              
              ImGui::Begin("Tracker");
              ImGui::Text("preX: %lf + %lf", target(0) ,target(3));
              ImGui::Text("preY: %lf + %lf", target(1) ,target(4));
              ImGui::Text("preZ: %lf", preZ);
              ImGui::Text("camX: %lf", cam(0));
              ImGui::Text("camY: %lf", cam(1) );
              ImGui::Text("camZ: %lf", cam(2) );
              ImGui::Text("yaw: %lf", rotAtt1.yaw);
              ImGui::Text("pitch: %lf", rotAtt1.pitch );
              ImGui::Text("distance: %lf", rotAtt1.distance );
              ImGui::End();
              Armor.camxyz_show=cam;

              rotAtt = xyz2PitchYawDis(cam(2), cam(0), cam(1));

              // 串口在此获取信息 yaw pitch distance，同时设定目标装甲板数字
              double yaw = rotAtt.yaw;
              float pitch = rotAtt.pitch;
              float distance = rotAtt.distance;
              float hr= target(2) * 0.001;
              // cout << ">>>>=== Serial::Get()->SerialSend @ " << CurrentPreciseTime() << " Yaw=" << yaw << ", Pitch=" << pitch << "\n";
              ImGui::Begin("HR");
              ImGui::Text("hr:%lf",hr);
              ImGui::End();


              if(yaw == NAN || pitch == NAN)
                  Serial::Get()->SerialSend(TxPrevYaw, TxPrevPitch, TxPrevDist, false, false, TxPrevArmorNum, SerialDeviceName);
              else {
                  TxPrevYaw = rotAtt1.yaw + 180 +0.5; TxPrevPitch = -bullet.selectalg(received.pitch ,distance * 0.001 , received.bullet_speed ,hr+0.03); TxPrevDist = rotAtt.distance * 0.001; TxPrevArmorNum = Armor.getTarget().armorNum;
                  Serial::Get()->SerialSend(TxPrevYaw, TxPrevPitch, TxPrevDist, targetValid, trackState.doFire, Armor.getTarget().armorNum, SerialDeviceName);
                  ImGui::Begin("Tracker");
                  ImGui::Text("TxPrevYaw: %lf", TxPrevYaw);
                  ImGui::Text("TxPrevPitch: %lf", TxPrevPitch );
                  ImGui::Text("TxPrevDist: %lf", TxPrevDist );
                  ImGui::End();
                  Armor.predxyz_show=Eigen::Vector3f{}; 
              }
          }
          else if(received.mode == 2)
          {
            Serial::Get()->SerialSend(TxPrevYaw, TxPrevPitch, TxPrevDist, false, false, TxPrevArmorNum, SerialDeviceName);
          }
            double t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
            ImGui::Begin("Info");
            ImGui::Text("Image acquiring FPS: %f", 1 / t1);
            ImGui::End();

#ifdef DEBUG_MODE
            Armor.showDebugInfo(1, 1, 1, 1);
#endif
            bool bRun = true;
            char chKey = waitKey(1);
        }
        //TODO：激光测距模式补全
        // else if(mode=Mode::LASER)
        // {
        //     TxPrevYaw = received.yaw; TxPrevPitch =bullet.selectalg(received.pitch, received.distance * 0.001, received.bullet_speed, received.pitch*received.distance); TxPrevDist = rotAtt.distance * 0.001; TxPrevArmorNum = Armor.getTarget().armorNum;
        //     Serial::Get()->SerialSend(TxPrevYaw, TxPrevPitch, TxPrevDist, false, false, TxPrevArmorNum, SerialDeviceName);
        // }

        ImguiDbgkit::get()->FinishFrame();
    }
    cerr << "armorDetectThread EXIT!" << endl;
    condVarMainThreadExit.notify_all(); // 让主线程退出
}
