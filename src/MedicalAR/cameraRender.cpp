#include "cameraRender.h"

vtkMyCommand::vtkMyCommand()
{
    image_vtk = cv::Mat(cv::Size(VtkPose::width, VtkPose::height), CV_8UC3);
}

void vtkMyCommand::setTheStyle(SelfInteractorStyle * interactorStyle_)
{
    interactorStyle = interactorStyle_;
}

void vtkMyCommand::Execute(vtkObject *caller, unsigned long eventId, void* callData)
{
    //press the "y" to start or stop
    if(interactorStyle->isRendering() == true)
    {
        std::unique_lock<std::mutex> lc(mMutexRender);
//        interactorStyle->castCamera(m_position,m_focalpoint,m_up);
        interactorStyle->castCamera(Rotation,Translation);
    }
    windowPrtSc();
}

void vtkMyCommand::setCameraInfo(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up)
{
    std::unique_lock<std::mutex> lc(mMutexRender);
    m_position = position;
    m_focalpoint = focalPoint;
    m_up = up;
}

void vtkMyCommand::setCameraInfo(Eigen::Matrix3d Rotation_, Eigen::Vector3d translation_)
{
    std::unique_lock<std::mutex> lc(mMutexRender);
    Rotation = Rotation_;
    Translation = translation_;
}

void vtkMyCommand::windowPrtSc()
{
    vtkSmartPointer<vtkWindowToImageFilter> windowto_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowto_image_filter->SetInput(interactorStyle->GetInteractor()->GetRenderWindow());
        windowto_image_filter->SetInputBufferTypeToRGB();
        windowto_image_filter->ReadFrontBufferOff();
        windowto_image_filter->Update();

    image1 = windowto_image_filter->GetOutput();

    std::lock_guard<std::mutex> lck(mMutexImageVtk);
    cv::Mat temp(cv::Size(interactorStyle->GetInteractor()->GetRenderWindow()->GetSize()[0],
                            interactorStyle->GetInteractor()->GetRenderWindow()->GetSize()[1]), CV_8UC3);
    temp.data=(unsigned char*)image1->GetScalarPointer();
    image_vtk = temp.clone();
}

void vtkMyCommand::setTheWindowToImageFilter(vtkWindowToImageFilter* windowto_image_filter)
{
    mWindowto_image_filter = windowto_image_filter;
}

cv::Mat vtkMyCommand::GetTheImageVtk()
{
    cv::Mat image;
    std::lock_guard<std::mutex> lck(mMutexImageVtk);

    if(!image_vtk.empty())
    {
        image= image_vtk.clone();
        cv::resize(image,image,cv::Size(VtkPose::width,VtkPose::height));
    }
    else
        image =  cv::Mat::zeros(VtkPose::height,VtkPose::width,CV_8UC3);

    return image.clone();
}
