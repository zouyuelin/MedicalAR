#ifndef PCLVISUAL_H
#define PCLVISUAL_H

#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

#include <sophus/se3.h>

#include <fstream>
#include <mutex>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class pclIcpProcess{

public:
    pclIcpProcess(std::string name);
    Eigen::Matrix4d icpProcess(Eigen::Matrix4f Twcr_rough);
    void setSlamMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map);
    void setPolydata(vtkPolyData* wSourceData_);

private:
    Sophus::SE3 finalTrans_Twcr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mSlamglobalMap;
    std::string inputName;
public:
    vtkPolyData* wSourceData;
};




#endif // PCLVISUAL_H


