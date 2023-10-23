 #include "../GxCamera/GxCamera.h"
#include "../General/General.h"
#include "General.h"
#include <condition_variable>


/*GxCamera camera;*/             // import Galaxy Camera
extern cv::Mat src;          // Transfering buffer

// RAII Object used to notify a condition variable when the scope is destroyed
class CondVarNotifier {
public:
	CondVarNotifier(std::condition_variable& cv) : _cv(cv) {}
	~CondVarNotifier() { _cv.notify_all(); }
private:
	std::condition_variable &_cv;
};


int imageUpdatingThreadCamera()
{
	CondVarNotifier notifier(condVarMainThreadExit);
	
	GX_STATUS status = GX_STATUS_SUCCESS;

	/*
	 *Preparation: CvMat image content
	*/
	cv::Mat frame;

	/*
	 *First init: Implementation of GxCamera and SetInitialArmor it
	*/
	GxCamera gxCam;
	status = gxCam.initLib();
	GX_VERIFY(status);

	/*
	 *Second SetInitialArmor: Open Camera by SN/Index
	*/
     //status = gxCam.openDeviceBySN(SN_sentry_below);	//By SN
    status = gxCam.openDeviceByIndex("1");		//By Index
	GX_VERIFY(status);

	/*
	 *Third SetInitialArmor: Set Camera Params: ROI, Exposure, Gain, WhiteBalance
	*/
    gxCam.setRoiParam(1280, 1024, 0, 0);				// ROI
    gxCam.setExposureParam(5000, false, 10000, 30000);	// Exposure
	gxCam.setGainParam(8, false, 0, 10);				// Gain
	gxCam.setWhiteBalanceOn(true);						// WhiteBalance
	/*
	 *Before acq: Send Acquisition Start Command
	*/
	status = gxCam.startAcquiring();					// Send Start Acquisition Command
	GX_VERIFY(status);
    cv::VideoWriter vi("./test.avi",cv::VideoWriter::fourcc('M','J','P','J'),25.0,Size(1080,1280));
    while (true)
    {
        // FPS
        double t = cv::getTickCount();
        /*
         *In acq: Snap a CvMat Image and store it in CvMat Content
        */
        status = gxCam.snapCvMat(frame);
        //GX_VERIFY(status);
        // 不要在这里VERIFY，如果打调试断点了这个地方就会超时，然后循环断掉
        if(status != GX_STATUS_SUCCESS) {
            GetErrorString(status);
            continue;
        }
        // Update the image acquired to src Mat content
        if (1) {
            unique_lock <mutex> lck(Globalmutex);
            frame.copyTo(src);
            imageReadable = true;
            FrameFetched.store(true);
            GlobalCondCV.notify_one();
        }
        vi.write(src);
    //		char chKey = waitKey(1);
    //		if (chKey == 'w' || chKey == 'W')
    //			break;
        //FPS
        double t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
       // printf("Image Acquiring FPS: %f\n", 1 / t1);
    }

	/*
	 *After acq: Send Acquisition Stop Command
	*/
	status = gxCam.stopAcquiring();
	GX_VERIFY(status);

	/*
	*Close camera, while you can still open a new different camera
	*/
	gxCam.closeDevice();

	/*
	*Close lib: you can not use any GxCamera device unless you initLib() again
	*/
	gxCam.closeLib();

    return status;
}

int imageUpdatingThreadLocal(std::string fileName)
{ 
	const char *filename = "/home/shaobing2/Desktop/outpost/Blue_Snipe_Halfspeed.mp4";
	Mat frame;
	cv::VideoCapture capture;

    if(!fileName.empty())
        filename = fileName.c_str();
    capture.open(filename);

	if (!capture.isOpened())
	{
		printf("can not open ... \n");
		return -1;
	}

	while (true)
	{
		bool singleFrameMode = SingleFrameMode.load(),
			 singleFrameFlag = false;
		singleFrameFlag = SingleFrameFlag.exchange(singleFrameFlag);
		if(!singleFrameMode || (singleFrameMode && singleFrameFlag)) {
			if(!capture.read(frame))
			{
				capture.set(cv::CAP_PROP_POS_FRAMES, 0);
				continue;
			}
			else
			{
				capture.retrieve(frame);
			}
			FrameFetched = true;
		}
		double t = cv::getTickCount();
		if (1) {
			unique_lock <mutex> lck(Globalmutex);
			frame.copyTo(src);
			//Mat show;
			//frame.copyTo(show);
			//imshow("frame", show);
			imageReadable = true;
			GlobalCondCV.notify_one();
		}
		// double t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
		// printf("Image Acquiring FPS: %f\n", 1 / t1);
		 waitKey(10);
	}
	capture.release();
	cerr << "imageUpdatingThreadLocal EXIT!" << endl;
	return 0;
}
