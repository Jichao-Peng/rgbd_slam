//
// Created by leo on 18-8-25.
//

#ifndef PROJECT_SLAM_BASE_H
#define PROJECT_SLAM_BASE_H

#include <omp.h>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>


using namespace cv;
using namespace std;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    float cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    int frame_id;
    cv::Mat image, depth; //该帧对应的彩色图与深度图
    cv::Mat descriptor;       //特征描述子
    vector<cv::KeyPoint> keypoint; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};


class SLAMBase
{
public:
    //生成点云
    void CreatCloud(const Mat &depth, const Mat &image, const CAMERA_INTRINSIC_PARAMETERS& camera ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void ParaCreatCloud(const Mat &depth, const Mat &image, const CAMERA_INTRINSIC_PARAMETERS& camera ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    //计算关键点和描述子
    void ComputeKeyPointAndDescriptor(FRAME& frame);

    //计算运动估计
    RESULT_OF_PNP EstimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

    //合成点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr JoinPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera );

    //旋转向量转齐次矩阵
    Eigen::Isometry3d RotVector2HomoMatrix( cv::Mat& rvec, cv::Mat& tvec );
    //旋转矩阵转齐次矩阵
    Eigen::Isometry3d RotMatrix2HomoMatrix( cv::Mat& R, cv::Mat& t );

    //计算运动范围
    double NormalTransform(Mat rvec, Mat tvec);



private:
    Point3f Point2dTo3d( Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

};


#endif //PROJECT_SLAM_BASE_H
