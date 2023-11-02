#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "Armor/Armor.h"
#include "GxCamera/GxCamera.h"
using std::thread;

// Muti-threads control variables
mutex Globalmutex;                        // C++11 mutex
condition_variable GlobalCondCV;          // C++11 condition variable
bool imageReadable = false;               // threads conflict due to image-updating
Mat src = Mat::zeros(1024, 1280, CV_8UC3);   // Transfering buffer
std::atomic_bool SingleFrameFlag(false), SingleFrameMode(false), FrameFetched(false);

mutex mutexMainThreadExit;
condition_variable condVarMainThreadExit;

int main(int argc, char** argv)
{
    bool doUseVideo = false;
    std::string videoFileName;
    if(argc > 1) {
        if(!strcmp(argv[1], "CaptureVideo")) {
            doUseVideo = true;
            if(argc > 2)
                videoFileName = argv[2];
        } else if(!strcmp(argv[1], "SerialReceiveDebug")) {
            Serial::Get()->EnableDebugOutput(true, false);
        }
    }

      // 创建线程
    if(doUseVideo)
      thread(imageUpdatingThreadLocal, videoFileName).detach();
    else
      thread(imageUpdatingThreadCamera).detach();
    thread(armorDetectingThread).detach();
    thread(Serial::ReceiveThreadTask).detach();
    thread(Serial::TransmitThreadTask).detach();

    // Main thread
	// while (true)
	// {
	// 	char chKey = getchar();
	// 	if (chKey == 'Q' || chKey == 'q')
    //         return 0;
	// }
	unique_lock<mutex> lck(mutexMainThreadExit);
	condVarMainThreadExit.wait(lck);
	return 0;
}
