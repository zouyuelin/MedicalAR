#ifndef VTKVISUAL_H
#define VTKVISUAL_H

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>

#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkAxesActor.h>
#include <vtkAxes.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCoordinate.h>
#include<vtkCamera.h>
#include "vtkPolyDataMapper.h"
#include "vtkLODActor.h"
#include "vtkCamera.h"
#include "vtkProperty.h"
#include "vtkSTLReader.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleImage.h"

#include "pclVisual.h"
#include "SelfInteractorStyle.h"
#include "cameraRender.h"

#include<iostream>
#include <unistd.h>
#include <thread>
#include <vector>
#include <memory>

using namespace std;

class vtkMyCommand;

class SelfInteractorStyle;


class VtkPose
{
public:
    VtkPose(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pts, std::string config);

    void run();
    void vtkRunner();
    void getpose(cv::Mat mMat);
    void getrgbDepth(cv::Mat image,cv::Mat depth);
    void combination();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SlamglobalMap;
    vtkPolyData* ptsPolydata;
    static Eigen::Isometry3d mTcwMat;

protected:
    Eigen::Matrix3d Rotate;
    Eigen::Vector3d Translation;

    std::mutex mMutex;
    std::mutex vtkImageDataMutex;

    cv::Mat image_vtk;
    cv::Mat image_slam;
    cv::Mat image_depth;

private:
    int imageSizeH = 640;
    int imageSizeW = 480;
    double ratio = 1000.0;//双目相机
    double mDepthMapFactor = 0.001;

    std::string inputFilename;
//    double ratio = 4000.0;//深度相机
    bool ok = false;

    pclIcpProcess * pclview;
    vtkMyCommand* mCommand = nullptr;
    vtkSmartPointer<SelfInteractorStyle> mStyle;

    std::shared_ptr<std::thread> pclThread;
    std::shared_ptr<thread> vtkRenderThread;
    std::shared_ptr<thread> vtkRenderThreadrh;
    std::shared_ptr<std::thread> sendInfolThread;

public:
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float width;
    static float height;
    static float focusl;
    string targetFile;
};

#endif // VTKPOSE_H
