
#include "vtkVisual.h"
#include <math.h>

using namespace std;

float VtkPose::fx = 0;
float VtkPose::fy = 0;
float VtkPose::cx = 0;
float VtkPose::cy = 0;
float VtkPose::width = 0;
float VtkPose::height = 0;

VtkPose::VtkPose(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pts, string config, string name)
{
    Rotate = (Eigen::Matrix3d::Identity());
    mTcwMat = (Eigen::Isometry3d::Identity());

    cv::FileStorage fSettings(config, cv::FileStorage::READ);
    fx= fSettings["Camera.fx"];
    fy= fSettings["Camera.fy"];
    cx= fSettings["Camera.cx"];
    cy= fSettings["Camera.cy"];
    width= fSettings["Camera.width"];
    height= fSettings["Camera.height"];

    image_vtk=cv::Mat(cv::Size(640, 480), CV_8UC3);
    image_slam=cv::Mat(cv::Size(640, 480), CV_8UC3);

    inputFilename = name;

//    //init the argvs
    pclview = new pclIcpProcess(inputFilename);
    mCommand = vtkMyCommand::New();
    mStyle = vtkSmartPointer<SelfInteractorStyle>::New();

    //init the pointer SlamglobalMap
    SlamglobalMap = pts;

    sendInfolThread = make_shared<thread>(bind(&VtkPose::run,this));  //camera information sending thread
    vtkRenderThread = make_shared<thread>(bind(&VtkPose::vtkRunner,this)); //vtk 主线程
    vtkRenderThreadrh = make_shared<thread>(bind(&VtkPose::combination,this)); //融合图像显示线程
}

void VtkPose::vtkRunner()
{
    ptsPolydata = vtkPolyData::New();

    // stl model visualization

    vtkSmartPointer<vtkSTLReader> reader =
        vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(inputFilename.c_str());
    reader->Update();

    pclview->setSlamMap(SlamglobalMap);
    pclview->setPolydata(reader->GetOutput());

    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapperPts =
        vtkSmartPointer<vtkPolyDataMapper>::New();
        mapperPts->SetInputData(ptsPolydata);

    vtkSmartPointer<vtkActor> actorPts =
        vtkSmartPointer<vtkActor>::New();
        actorPts->SetMapper(mapperPts);
        actorPts->GetProperty()->SetColor(1.0,0.49,0.25);
        actorPts->SetVisibility(1);

    vtkSmartPointer<vtkActor> actor =
        vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(1.0,0.49,0.25);

    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(100,100,100);

    vtkSmartPointer<vtkCoordinate> coordinate = vtkSmartPointer<vtkCoordinate>::New();
        coordinate->SetCoordinateSystemToNormalizedDisplay();
        coordinate->SetValue(0.5,0.5,0);

    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
        renderer->AddActor(actor);
        renderer->AddActor(actorPts);
//    renderer->AddActor(axes);//添加坐标轴

    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
        renWin->AddRenderer(renderer);
        renWin->SetSize(640,480);

    vtkSmartPointer<vtkWindowToImageFilter> windowto_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowto_image_filter->SetInput(renWin);
        windowto_image_filter->SetInputBufferTypeToRGB();
        windowto_image_filter->ReadFrontBufferOff();
//        windowto_image_filter->Update();

    vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
            iren->SetInteractorStyle(mStyle);
            iren->SetRenderWindow(renWin);

        iren->Initialize();
        renderer->ResetCamera();

        //init the vtkcommand
        mCommand->setTheStyle(mStyle);
        mCommand->setTheWindowToImageFilter(windowto_image_filter);

        //add an observer
        iren->AddObserver(vtkCommand::TimerEvent, mCommand);
        iren->CreateRepeatingTimer(5);

        mStyle->setTheArgs(ptsPolydata,SlamglobalMap);
        mStyle->setTheicpProcess(pclview);
        //start the events loop
        iren->Start();
}

void VtkPose::run()
{
    while(true)
    {
        {
            //send the information of the posture
            std::unique_lock<std::mutex> lc(mMutex);
            mCommand->setCameraInfo(Rotate,Translation);
        }
        usleep(10*1000);
    }

    return;
}

void VtkPose::getpose(cv::Mat mMat)
{
    try
    {
        Eigen::Matrix4d mTcw;
        cv::cv2eigen(mMat,mTcw);
        {
            std::unique_lock<std::mutex> lc(mMutex);
            mTcwMat = Eigen::Isometry3d(mTcw);
            Rotate = mTcwMat.rotation();
            Translation = mTcwMat.translation()*ratio;
        }

    }
    catch(...)
    {

    }
}
void VtkPose::getrgb(cv::Mat image){
    std::unique_lock<std::mutex> lc(vtkImageDataMutex);
    image_slam=image.clone();
}

void VtkPose::combination(){

    cv::Mat image_rh(cv::Size(640, 480), CV_8UC3);
    cv::Mat image_slam_;
    cv::Mat image_vtk_;
    cv::Mat image_vtk_xz(cv::Size(640, 480), CV_8UC3);
    cv::Mat image_final;
    while (true) {

    std::unique_lock<std::mutex> lc(vtkImageDataMutex);
    image_slam_ = image_slam.clone();
    lc.unlock();

    cv::resize(image_slam_,image_slam_,cv::Size(640,480));

        image_vtk_ = mCommand->GetTheImageVtk();

    cv::flip(image_vtk_,image_vtk_xz,-1);


    for (int i = 0; i < image_slam_.rows; ++i)
    {
        cv::Vec3b *praw = image_slam_.ptr<cv::Vec3b>(i);
        cv::Vec3b *pvtk = image_vtk_xz.ptr<cv::Vec3b>(i);
        cv::Vec3b *prh = image_rh.ptr<cv::Vec3b>(i);
        for (int j = 0; j < image_slam_.cols; ++j)
        {
            if (pvtk[image_slam_.cols-j-1][0]!=0)
            {
//BG2RGB
                prh[j][0]=praw[j][0]*0.4+pvtk[image_slam_.cols-j-1][2]*0.6;
                prh[j][1]=praw[j][1]*0.4+pvtk[image_slam_.cols-j-1][1]*0.6;
                prh[j][2]=praw[j][2]*0.4+pvtk[image_slam_.cols-j-1][0]*0.6;
            }
            else {
                prh[j][0]=praw[j][0];
                prh[j][1]=praw[j][1];
                prh[j][2]=praw[j][2];
            }

        }
    }

    cv::resize(image_rh,image_final,cv::Size(image_rh.cols*2,image_rh.rows*2));
    cv::imshow("combination_image",image_final);
    cv::waitKey(20);

    }
}
