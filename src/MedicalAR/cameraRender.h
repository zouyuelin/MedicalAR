#ifndef CAMERARENDER_H
#define CAMERARENDER_H

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <mutex>

#include "vtkWindowToImageFilter.h"
#include "SelfInteractorStyle.h"
#include "vtkVisual.h"

class SelfInteractorStyle;
class VtkPose;

class vtkMyCommand : public vtkCommand
{
public:
    vtkMyCommand();
    vtkTypeMacro(vtkMyCommand,vtkCommand);
    static vtkMyCommand *New()
    {
        return new vtkMyCommand;
    }
    virtual void Execute(vtkObject *caller, unsigned long eventId, void* callData);
    void setCameraInfo(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up);
    void setCameraInfo(Eigen::Matrix3d Rotation_,Eigen::Vector3d translation_);
    void setTheStyle(SelfInteractorStyle * interactorStyle_);
    void setTheWindowToImageFilter(vtkWindowToImageFilter* windowto_image_filter);
    cv::Mat GetTheImageVtk();
    vtkWindowToImageFilter* mWindowto_image_filter;

protected:
    void windowPrtSc();
private:
    SelfInteractorStyle * interactorStyle = nullptr;
    Eigen::Vector3d m_position;
    Eigen::Vector3d m_focalpoint;
    Eigen::Vector3d m_up;

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Translation;
    cv::Mat image_vtk;
    std::mutex mMutexImageVtk;
    std::mutex mMutexRender;
    vtkSmartPointer<vtkImageData> image1;
};

#endif // CAMERARENDER_H
