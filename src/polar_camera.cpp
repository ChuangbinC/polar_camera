#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;

class PolarCamera
{

public:
    PolarCamera();
    ~PolarCamera();

    inline bool isValid() const { return isValidFlag; }

    bool setCameraParameters(CameraPtr pCam, INodeMap &nodeMap, INodeMap &nodeMapTLDevice);
    bool AcquireImages(CameraPtr pCam, Mat &image);
    void startAcquisition();
    void getImage(Mat &image);

private:
    CameraPtr pCam = nullptr;
    SystemPtr system;
    CameraList camList;
    bool isValidFlag = false;
    bool isAcquisition = false;
};

PolarCamera::PolarCamera()
{

    system = System::GetInstance();

    camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl
         << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        cout << "Not enough cameras!" << endl;
        isValidFlag = false;
        return;
    }

    isValidFlag = true;

    pCam = camList.GetByIndex(0);

    pCam->Init();

    INodeMap &nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    INodeMap &nodeMap = pCam->GetNodeMap();
    setCameraParameters(pCam, nodeMap, nodeMapTLDevice);

}

PolarCamera::~PolarCamera()
{
    if (isAcquisition)
        pCam->EndAcquisition();
    if (isValidFlag)
        pCam->DeInit();
    pCam = nullptr;
    camList.Clear();
    system->ReleaseInstance();
    cout << "release all source!"<<endl;
}

void PolarCamera::startAcquisition()
{
    // *** NOTES ***
    // 相机开始采集图像时会发生什么情况取决于采集模式。
    // 单帧仅捕获单个图像，多帧则捕获一定数量的图像，并连续捕获连续的图像流。 由于该示例要求检索10张图像，因此已设置连续模式。

    cout << "Acquiring images..." << endl;
    pCam->BeginAcquisition();
    isAcquisition = true;
}

bool PolarCamera::setCameraParameters(CameraPtr pCam, INodeMap &nodeMap, INodeMap &nodeMapTLDevice)
{
    try
    {
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl
                 << endl;
            return -1;
        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl
                 << endl;
            return -1;
        }

        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        cout << "Acquisition mode set to continuous..." << endl;

        return true;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Set parameters failed. Error: " << e.what() << endl;
        return false;
    }
}

void PolarCamera::getImage(Mat &image)
{

    AcquireImages(pCam, image);
}

bool PolarCamera::AcquireImages(CameraPtr pCam, Mat &image)
{
    try
    {
        ImagePtr pResultImage = pCam->GetNextImage();
        if (pResultImage->IsIncomplete())
        {
            cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                 << "..." << endl
                 << endl;
        }
        else
        {
            const size_t width = pResultImage->GetWidth();
            const size_t height = pResultImage->GetHeight();
            ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

            unsigned int row_size = convertedImage->GetHeight();

            unsigned int col_size = convertedImage->GetWidth();

            unsigned int num_channels = convertedImage->GetNumChannels();

            Mat current_frame = cv::Mat(row_size, col_size,

                                        (num_channels == 3) ? CV_8UC3 : CV_8UC1,

                                        convertedImage->GetData(), convertedImage->GetStride());
            current_frame.copyTo(image);
            // cout << "Polar image fisrt data is " << result[0] << endl;
            pResultImage->Release();
        }
        
        return true;
    }

    catch (Spinnaker::Exception &e)
    {
        if (!isAcquisition){
            cout << "Error: Camera doesn't start acquisition. Please use \"startAcquisition\" function to start it.";
            return false;
        }
            
        cout << "Capture image failed. Error: " << e.what() << endl;
        return false;
    }
}

// https://blog.csdn.net/baidu_18189515/article/details/52332754
// https://blog.csdn.net/github_30605157/article/details/50990493
int main(int argc, char **argv)
{
    ros::init(argc, argv, "polar_camera");
    PolarCamera polarcamera;
    if (!polarcamera.isValid())
        return -1;
    polarcamera.startAcquisition();
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // 第二个是缓冲区的大小（即消息队列的长度，在发布图像消息时消息队列的长度只能是1）。
    image_transport::Publisher pub = it.advertise("polarcamera/image", 1);

    //设置主题的发布频率为30Hz
    ros::Rate loop_rate(30);
    while (nh.ok())
    {
        cv::Mat image;
        polarcamera.getImage(image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        //按照设定的频率来将程序挂起
        loop_rate.sleep();
    }
    return (0);
}