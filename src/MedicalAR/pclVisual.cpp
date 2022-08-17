#include "pclVisual.h"
#include <thread>

// 用于将参数传递给回调函数的结构体
pclIcpProcess::pclIcpProcess(std::string name)
{
    inputName = name;
}

void pclIcpProcess::setSlamMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map){
    mSlamglobalMap = Map;
}

void pclIcpProcess::setPolydata(vtkPolyData* wSourceData_)
{
    wSourceData = wSourceData_;
}

Eigen::Matrix4d pclIcpProcess::icpProcess(Eigen::Matrix4f Twcr_rough) {

    clock_t start = clock();

    PointCloudT::Ptr stl_cloud_W(new PointCloudT);
    pcl::io::vtkPolyDataToPointCloud(wSourceData,*stl_cloud_W);

    PointCloudT::Ptr registRough(new PointCloudT);
    pcl::transformPointCloud(*mSlamglobalMap, *registRough, Twcr_rough);
    //mSlamglobalMap = Cr, registRough = W_rough, Twcr_rough * mSlamglobalMap=registRough

    pcl::PointCloud<PointT>::Ptr registSmooth(new pcl::PointCloud<PointT>);
    //icp精密配准
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    // 最大迭代次数
    icp.setMaximumIterations(500);
    // 对应距离
    icp.setMaxCorrespondenceDistance (0.09);
    // 两次变化矩阵之间的差值
    icp.setTransformationEpsilon(1e-10);
    // 均方误差
    icp.setEuclideanFitnessEpsilon(0.01);

    icp.setInputSource(registRough);//录入source点云 W_rough
    icp.setInputTarget(stl_cloud_W);//录入target点云 W

    icp.align(*registSmooth);//最终配准结果

    Eigen::Matrix4f icp_trans;
    icp_trans = icp.getFinalTransformation();
    //icp_trans*registRough = stl_cloud_W
    //so the last transformed matrix is : icp_trans * Twcr_rough  = Twcr;

    clock_t end = clock();
    cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;//输出配准所用时间

    Eigen::Matrix4f Twcr_f = icp_trans*Twcr_rough;
    Eigen::Matrix4d Twcr = Twcr_f.cast<double>();
    return Twcr;
}
