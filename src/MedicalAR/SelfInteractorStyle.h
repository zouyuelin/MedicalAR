#ifndef INTERACTORSTYLE_H
#define INTERACTORSTYLE_H

#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointData.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkCoordinate.h>
#include <vtkKdTreePointLocator.h>
#include <vtkSphereSource.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkActorCollection.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkKochanekSpline.h>
#include <vtkCamera.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <sophus/se3.h>

#include "pclVisual.h"
#include "vtkVisual.h"

#include <mutex>

class pclRender;

class VtkPose;

class SelfInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static SelfInteractorStyle* New();
    SelfInteractorStyle();
    vtkTypeMacro(SelfInteractorStyle, vtkInteractorStyleTrackballCamera);

    bool isRendering();
    void castCamera(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up);
    void castCamera(Eigen::Matrix3d Rotate,Eigen::Vector3d translation);
    void setTheArgs(vtkPolyData* ptsPolydata_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SlamgPoints);
    void setTheCameraPose(Eigen::Isometry3d p);
    void setTheicpProcess(pclIcpProcess* icp_);
    void addRealPoints(double x, double y, double z);

    void OnKeyPress();
    void OnLeftButtonDown();

    std::vector<cv::Point2f> realPoints;

protected:
    void SetTheInitPostion(Sophus::SE3 Twcr);
    Sophus::SE3 GuassNewtomIter();

private:
    Eigen::Vector3d mInitPostion;
    Eigen::Matrix3d mInitPosture;
    Sophus::SE3 initSe3;
    bool Rendering = false;

    std::map<unsigned int, vtkActor*> seedList;
    std::map<unsigned int, bool> seedTypes;
    std::vector<Eigen::Vector3d> vtkSelectPoints;
    std::vector<Eigen::Vector3d> pclSelectPoints;

    unsigned int m_numOfSeeds;
    bool ptsType = false; //false presents virtul model, true presents real model

    vtkPolyData* ptsPolydata;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mSlamgPoints;

    vtkActor* last_transActor = nullptr;
    pclIcpProcess* icp = nullptr;

    std::mutex RenderMutex;

    Eigen::Isometry3d poseCamera;
};

#endif // INTERACTORSTYLE_H
