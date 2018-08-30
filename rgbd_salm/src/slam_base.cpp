//
// Created by leo on 18-8-25.
//

#include "slam_base.h"
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SLAMBase::JoinPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, FRAME &new_frame, Eigen::Isometry3d T,
                                                            CAMERA_INTRINSIC_PARAMETERS &camera)
{
    clock_t time_a,time_b,time_c,time_d;
    time_a = clock();

    //生成新的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    CreatCloud( new_frame.depth, new_frame.image, camera, new_cloud );


    time_b = clock();

    //旋转点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud( *new_cloud, *trans_cloud, T.matrix() );

    time_c = clock();

    //合成点云
    *original_cloud += *trans_cloud;

    time_d = clock();

//    cout<<(double)(time_b-time_a)/CLOCKS_PER_SEC<<" "
//    <<(double)(time_c-time_b)/CLOCKS_PER_SEC<<" "
//    <<(double)(time_d-time_c)/CLOCKS_PER_SEC<<endl<<endl;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setLeafSize( 0.02, 0.02, 0.02 );
    voxel.setInputCloud( original_cloud );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
    voxel.filter( *filter_cloud );
    return filter_cloud;
}

//生成点云
void SLAMBase::CreatCloud(const Mat &depth, const Mat &image, const CAMERA_INTRINSIC_PARAMETERS& camera ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    for (int m = 0; m < depth.rows; m+=2)
        for (int n=0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZRGB p;

            // 计算这个点的空间坐标
            p.z = float(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = image.ptr<uchar>(m)[n*3];
            p.g = image.ptr<uchar>(m)[n*3+1];
            p.r = image.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    cloud->header.frame_id = "map";
}

//用显存并行计算的方式计算点云，比较是否加快计算速度
//TODO:点云合成有问题，测试的话需要修改cmaklist
void SLAMBase::ParaCreatCloud(const Mat &depth, const Mat &image, const CAMERA_INTRINSIC_PARAMETERS& camera ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    cloud->points.resize(depth.cols*depth.rows/4);
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    cloud->header.frame_id = "map";

#pragma omp parallel for
    for (int m = 0; m < depth.rows; m+=2)
    {
        pcl::PointXYZRGB *itp = &cloud->points[m/2 * (depth.cols/2)];
        const ushort *itd = depth.ptr<ushort>(m);
//        printf("j = %d, ThreadId = %d\n", m, omp_get_thread_num());
        for (int n = 0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = itd[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;

            // 计算这个点的空间坐标
            itp->z = float(d) / camera.scale;
            itp->x = (n - camera.cx) * itp->z / camera.fx;
            itp->y = (m - camera.cy) * itp->z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            itp->b = image.ptr<uchar>(m)[n * 3];
            itp->g = image.ptr<uchar>(m)[n * 3 + 1];
            itp->r = image.ptr<uchar>(m)[n * 3 + 2];
        }
    }
}

//计算关键点和描述子
void SLAMBase::ComputeKeyPointAndDescriptor(FRAME &frame)
{
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

    detector = FeatureDetector::create( "ORB" );
    descriptor = DescriptorExtractor::create( "ORB" );

    detector->detect( frame.image, frame.keypoint );
    descriptor->compute( frame.image, frame.keypoint, frame.descriptor );
}

//估计运动
RESULT_OF_PNP SLAMBase::EstimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    //关键点进行匹配
    vector< DMatch > matches;
    BFMatcher matcher;
    matcher.match( frame1.descriptor, frame2.descriptor, matches );

    //筛选特征点
    RESULT_OF_PNP result;
    vector< DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = 10;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    if ( minDis < 10 )
        minDis = 10;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }

    //如果好的匹配点太少则放弃
    if (goodMatches.size() <= 5)
    {
        result.inliers = -1;
        cout<<"too few good matches, this frame is skipped"<<endl;
        return result;
    }

    // 第一个帧的三维点
    vector<Point3f> pts_obj;
    // 第二个帧的图像点
    vector< Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个

        // 提取第一帧图像的匹配点
        Point2f p = frame1.keypoint[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;//如果匹配点深度为零则略过
        // 将(u,v,d)转成(x,y,z)
        Point3f pt ( p.x, p.y, d );
        Point3f pd = Point2dTo3d( pt, camera );
        pts_obj.push_back( pd );

        //提取第二帧的匹配点
        pts_img.push_back( Point2f( frame2.keypoint[goodMatches[i].trainIdx].pt ) );
    }

    //如果匹配点数量为零的话略过
    //TODO:应该不是为零，而是小于某个值就要略过，pnp至少三个匹配点
    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        cout<<"too few points, this frame is skipped"<<endl;
        return result;
    }

    double camera_matrix_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
    };

    // 构建相机矩阵
    Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    Mat rvec, tvec, inliers;

    // 求解pnp
    solvePnPRansac( pts_obj, pts_img, cameraMatrix, Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}

//2d点转3d
Point3f SLAMBase::Point2dTo3d( Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    Point3f p; // 3D 点
    p.z = float( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

//旋转向量转齐次矩阵
Eigen::Isometry3d SLAMBase::RotVector2HomoMatrix( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ )
            r(i,j) = R.at<double>(i,j);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

//旋转矩阵转齐次矩阵
Eigen::Isometry3d SLAMBase::RotMatrix2HomoMatrix(cv::Mat &R, cv::Mat &t)
{
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ )
            r(i,j) = R.at<double>(i,j);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = t.at<double>(0,0);
    T(1,3) = t.at<double>(1,0);
    T(2,3) = t.at<double>(2,0);
    return T;
}

double SLAMBase::NormalTransform(Mat rvec, Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}