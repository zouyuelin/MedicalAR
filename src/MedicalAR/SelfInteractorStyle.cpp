#include "SelfInteractorStyle.h"
#include <math.h>

SelfInteractorStyle::SelfInteractorStyle():m_numOfSeeds(0)
{

}

void SelfInteractorStyle::OnLeftButtonDown()
{
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void SelfInteractorStyle::OnKeyPress()
{
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    if (key == "space")
    {

        this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
                this->Interactor->GetEventPosition()[1],
                0,  // always zero.
                this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

        double picked[3];
        this->Interactor->GetPicker()->GetPickPosition(picked);

        if(abs(picked[0])+abs(picked[1])+abs(picked[2])<0.01)
            return;

        m_numOfSeeds++;

        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetRadius(2.0);

            sphereSource->SetCenter(picked);

        std::cout<<"The select point in vtk is, x = "<<picked[0]<<" y = "<<picked[1]<<" z = "<<picked[2]<<std::endl;

        vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

        vtkActor* sphereActor = vtkActor::New();
            sphereActor->SetMapper(sphereMapper);

       seedList.insert(std::pair<unsigned int, vtkActor*>(m_numOfSeeds, sphereActor));
       seedTypes.insert(std::pair<unsigned int, bool>(m_numOfSeeds,ptsType));

        if(!ptsType)
        {
            vtkSelectPoints.push_back(Eigen::Vector3d(picked[0],picked[1],picked[2]));
            sphereActor->GetProperty()->SetColor(1, 0, 0);
        }
        else
        {
            pclSelectPoints.push_back(Eigen::Vector3d(picked[0],picked[1],picked[2]));
            //transforming Pw --> Pc, Tcw*Pw = Pc

            Eigen::Vector3d pc = poseCamera * Eigen::Vector3d(picked[0],picked[1],picked[2]);

            double x = pc[0]/(float)pc[2];
            double y = pc[1]/(float)pc[2];
            double u,v;
            u = x * VtkPose::fx + VtkPose::cx;
            v = y * VtkPose::fy + VtkPose::cy;
            realPoints.push_back(cv::Point2f(u,v));
            std::cout<<"u is :"<<u<<" v is:"<<v<<std::endl;
            sphereActor->GetProperty()->SetColor(0, 1, 0);
        }

        this->GetCurrentRenderer()->AddActor(sphereActor);

        this->GetInteractor()->Render();
    }
    else if (key == "y")
    {
        std::unique_lock<std::mutex> lc(RenderMutex);
        Rendering = !Rendering;
        this->GetInteractor()->GetRenderWindow()->SetSize(VtkPose::width,VtkPose::height);
        if(!Rendering)
        {
           this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
            vtkActor* actor = vtkActor::SafeDownCast(this->GetCurrentRenderer()->GetActors()->GetItemAsObject(1));
            vtkActor* actor1 = vtkActor::SafeDownCast(this->GetCurrentRenderer()->GetActors()->GetItemAsObject(0));
            actor->SetVisibility(1);
            actor1->SetVisibility(1);
            last_transActor->SetVisibility(1);
//            for(auto i = 0; i<seedList.size();i++)
//                seedList[i]->SetVisibility(1);
        }
        else
        {
            this->GetInteractor()->GetRenderWindow()->SetSize(VtkPose::width,VtkPose::height);
            vtkActor* actor = vtkActor::SafeDownCast(this->GetCurrentRenderer()->GetActors()->GetItemAsObject(1));
            vtkActor* actor1 = vtkActor::SafeDownCast(this->GetCurrentRenderer()->GetActors()->GetItemAsObject(0));
            actor->SetVisibility(0);
            actor1->SetVisibility(0);
            last_transActor->SetVisibility(0);
//            for(auto i = 0; i<seedList.size();i++)
//                seedList[i]->SetVisibility(0);
        }
        this->GetInteractor()->Render();
    }
    else if (key == "c")
    {
        ptsType = !ptsType;
        if(!ptsType)
            std::cout<<"The current type is virtul!"<<std::endl;
        else
            std::cout<<"The current type is real!"<<std::endl;
    }
    else if (key == "f")
    {
        pcl::io::pointCloudTovtkPolyData(*mSlamgPoints,ptsPolydata);
        std::cout<<ptsPolydata->GetPoints()->GetNumberOfPoints()<<std::endl;
        this->GetInteractor()->Render();
    }
    else if (key == "i")
    {
        Sophus::SE3 Twcr = GuassNewtomIter();
        std::cout<<"The transform matrix is : " << std::endl << Twcr.matrix() <<std::endl;
        //init the postion
        Eigen::Matrix4f Twc_rough_f = Twcr.matrix().cast<float>();

        Eigen::Matrix4d Twc_final = icp->icpProcess(Twc_rough_f);// =

        //Test the third point in vtkPoints real position and desir position.
//        Eigen::Vector3d test_ = vtkSelectPoints[2];
//        Eigen::Vector4d pc = poseCamera.matrix() * Twc_final.inverse() * Eigen::Vector4d(test_[0],test_[1],test_[2],1);
//        double x = pc[0]/(float)pc[2];
//        double y = pc[1]/(float)pc[2];
//        double u,v;
//        u = x * VtkPose::fx + VtkPose::cx;
//        v = y * VtkPose::fy + VtkPose::cy;
//        std::cout<<"The test point u,v is:" <<cv::Point2f(u,v)<<std::endl;

        Sophus::SE3 Twcr_final_se3(Twc_final.block(0,0,3,3),Twc_final.block(0,3,3,1));
        std::cout<<"The transform matrix smooth is : " << std::endl << Twcr_final_se3.matrix() <<std::endl;
        SetTheInitPostion(Twcr_final_se3);

        //show the  performance of registion
        vtkNew<vtkPolyData> transPolydata;
            transPolydata->DeepCopy(icp->wSourceData);
        vtkSmartPointer<vtkPolyDataMapper> transmapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
            transmapper->SetInputData(transPolydata);
        vtkSmartPointer<vtkActor> transactor =
                vtkSmartPointer<vtkActor>::New();
            transactor->SetMapper(transmapper);
            transactor->GetProperty()->SetColor(1.0,0,0);

        Eigen::Matrix4d TwcrE = Twcr_final_se3.inverse().matrix();

        vtkNew<vtkMatrix4x4> transMatrix;
        for(int i = 0; i<4;i++)
            for(int j = 0; j < 4; j++)
                transMatrix->SetElement(i,j,TwcrE(i,j));

        vtkNew<vtkTransform> transform;
            transform->SetMatrix(transMatrix);

        transactor->SetUserTransform(transform);

        this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(transactor);

        if(last_transActor != nullptr)
            this->GetCurrentRenderer()->RemoveActor(last_transActor);
        last_transActor = transactor;

        this->GetInteractor()->Render();
    }
    else if (key == "z")
    {
        if(m_numOfSeeds > 0)
        {
            seedList[m_numOfSeeds]->SetVisibility(0);
            this->GetCurrentRenderer()->RemoveActor(seedList[m_numOfSeeds]);

            if(!seedTypes[m_numOfSeeds])
            {
                vtkSelectPoints.erase(--vtkSelectPoints.end());
            }
            else
            {
                pclSelectPoints.erase(--pclSelectPoints.end());
                if(!realPoints.empty()) realPoints.pop_back();
            }

            seedList.erase(--seedList.end());
            seedTypes.erase(--seedTypes.end());

            m_numOfSeeds--;
            std::cout<<"The real is:"<<pclSelectPoints.size()<<" The vitul size is :"<<vtkSelectPoints.size()<<std::endl;
        }

        this->GetInteractor()->Render();
    }
}

void SelfInteractorStyle::castCamera(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up)
{
    vtkCamera* aCamera = vtkCamera::SafeDownCast(this->GetCurrentRenderer()->GetActiveCamera());

    aCamera->SetPosition ( position[0], position[1], position[2] );
    aCamera->SetFocalPoint( focalPoint[0], focalPoint[1], focalPoint[2] );
    aCamera->SetViewUp ( up[0], up[1], up[2] );

//    aCamera->ComputeViewPlaneNormal();
    //It is important to reset the camera clipping range.
    this->CurrentRenderer->ResetCameraClippingRange();
    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->SetActiveCamera(aCamera);

//    this->Interactor->GetRenderWindow()->Render();
    this->Interactor->Render();
}

void SelfInteractorStyle::castCamera(Eigen::Matrix3d Rotate,Eigen::Vector3d translation)
{
    vtkCamera* aCamera = vtkCamera::SafeDownCast(this->GetCurrentRenderer()->GetActiveCamera());

    Sophus::SE3 currentPos = initSe3 * Sophus::SE3(Rotate,translation).inverse();//Twcr * Tcnc1.inverse = Twcn

    Eigen::Vector3d t = currentPos.translation();
    Eigen::Matrix3d R = currentPos.rotation_matrix();


    Eigen::Vector3d eye,center,up;
        eye = t; //Twc,Rotate = Rcw, Translation = tcw, -Rcw^T * tcw(Pcw) = twc
        center = eye + R.col(2)*VtkPose::focusl; //Nzcw',相机z朝向
        up = -R.col(1);             //Nycw'，相机y朝向，这里viewup是反的

    aCamera->SetPosition ( eye[0], eye[1], eye[2] );
    aCamera->SetFocalPoint( center[0], center[1], center[2] );
    aCamera->SetViewUp ( up[0], up[1], up[2] );

    double wcx = -2*(VtkPose::cx-VtkPose::width/2) / VtkPose::width;
    double wcy = -2*(VtkPose::cy-VtkPose::height/2) / VtkPose::height;
    double viewAngle = 2.0 * std::atan2(VtkPose::height/2.0,VtkPose::fy) * 180.d / M_PI;
    aCamera->SetWindowCenter(wcx,wcy);
    aCamera->SetViewAngle(viewAngle);

//    aCamera->ComputeViewPlaneNormal();
    //It is important to reset the camera clipping range.
    this->CurrentRenderer->ResetCameraClippingRange();
    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->SetActiveCamera(aCamera);

//    this->Interactor->GetRenderWindow()->Render();
    this->Interactor->Render();
}

Sophus::SE3 SelfInteractorStyle::GuassNewtomIter()
{
    Sophus::SE3 Twcr;
    if(vtkSelectPoints.size() != pclSelectPoints.size())
    {
        std::cout<<"Please check the points selected !"<<std::endl;
        return Twcr;
    }

    double eps = 0.000001;
    for(int iter = 0; iter<30; iter++)
    {
        Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
        Eigen::Matrix<double,6,1> b = Eigen::Matrix<double,6,1>::Zero();
        for(int i = 0; i < vtkSelectPoints.size(); i++)
        {
            Eigen::Vector3d pw = Twcr.rotation_matrix() * pclSelectPoints[i] + Twcr.translation();
            Eigen::Vector3d e = pw - vtkSelectPoints[i];
            double loss = e.squaredNorm();
            Eigen::Matrix<double,3,6> J;
            J<<1, 0, 0,      0,  pw[2], -pw[1],
               0, 1, 0, -pw[2],      0,  pw[0],
               0, 0, 1,  pw[1], -pw[0],      0;

            H += J.transpose()*J;
            b += -J.transpose()*e;
         }
        Eigen::Matrix<double,6,1> dx;
        dx = H.ldlt().solve(b);
        Twcr = Sophus::SE3::exp(dx) * Twcr;

        if(dx.norm() < eps)
            break;
    }
    return Twcr;
}

void SelfInteractorStyle::addRealPoints(double x, double y, double z){

    ptsType = true;
    double picked[3] = {x, y, z};
    m_numOfSeeds++;

    vtkSphereSource* sphereSource = vtkSphereSource::New();
        sphereSource->SetRadius(2.0);

        sphereSource->SetCenter(picked);

    std::cout<<"The select point in vtk is, x = "<<picked[0]<<" y = "<<picked[1]<<" z = "<<picked[2]<<std::endl;

    vtkPolyDataMapper* sphereMapper = vtkPolyDataMapper::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkActor* sphereActor = vtkActor::New();
        sphereActor->SetMapper(sphereMapper);

    seedList.insert(std::pair<unsigned int, vtkActor*>(m_numOfSeeds, sphereActor));
    seedTypes.insert(std::pair<unsigned int, bool>(m_numOfSeeds,ptsType));

    if(!ptsType)
    {
        vtkSelectPoints.push_back(Eigen::Vector3d(picked[0],picked[1],picked[2]));
        sphereActor->GetProperty()->SetColor(1, 0, 0);
    }
    else
    {
        pclSelectPoints.push_back(Eigen::Vector3d(picked[0],picked[1],picked[2]));
        sphereActor->GetProperty()->SetColor(0, 1, 0);
    }

    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(sphereActor);

}

void SelfInteractorStyle::SetTheInitPostion(Sophus::SE3 Twcr)
{
    mInitPostion = Twcr.translation(); //twc, Rwc
    mInitPosture = Twcr.rotation_matrix();
    initSe3 = Twcr;
}

bool SelfInteractorStyle::isRendering()
{
    std::unique_lock<std::mutex> lc(RenderMutex);
    return Rendering;
}

void SelfInteractorStyle::setTheicpProcess(pclIcpProcess* icp_)
{
    icp = icp_;
}

void SelfInteractorStyle::setTheArgs(vtkPolyData* ptsPolydata_,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SlamgPoints)
{
    ptsPolydata = ptsPolydata_;
    mSlamgPoints = SlamgPoints;
}

void SelfInteractorStyle::setTheCameraPose(Eigen::Isometry3d p){
  poseCamera = p;
}

vtkStandardNewMacro(SelfInteractorStyle);
