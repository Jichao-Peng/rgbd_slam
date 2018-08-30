//
// Created by leo on 18-8-25.
//

#ifndef PROJECT_SLAM_H
#define PROJECT_SLAM_H

#include <time.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>


//#include <g2o/types/slam3d/types_slam3d.h>
//#include <g2o/core/sparse_optimizer.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/factory.h>
//#include <g2o/core/optimization_algorithm_factory.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>
//#include <g2o/core/robust_kernel.h>
//#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>

#include "slam_base.h"

using namespace std;
using namespace cv;

enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

class SLAM
{
public:
    SLAM();
    ~SLAM();
    void Run();

private:
    SLAMBase* slam_base;
    CAMERA_INTRINSIC_PARAMETERS camera;

    Mat GetImage(int frame_num);
    Mat GetDepth(int frame_num);

//    //检查关键帧
//    CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
//    // 检测近距离的回环
//    void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
//    // 随机检测回环
//    void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
};



#endif //PROJECT_SLAM_H
