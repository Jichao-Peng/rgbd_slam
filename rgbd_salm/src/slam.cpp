//
// Created by leo on 18-8-25.
//

#include "slam.h"

SLAM::SLAM()
{
    slam_base = new SLAMBase;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    camera.scale = 1000.0;
}

SLAM::~SLAM()
{
    delete slam_base;
}

Mat SLAM::GetImage(int frame_num)
{
    char filename[100];
    sprintf(filename,"/home/leo/Desktop/learn_rgbdsalm/data/rgb_png/%d.png",frame_num);
    Mat image = imread(filename);
    return image;
}

Mat SLAM::GetDepth(int frame_num)
{
    char filename[100];
    sprintf(filename,"/home/leo/Desktop/learn_rgbdsalm/data/depth_png/%d.png",frame_num);
    Mat depth = imread(filename);
    return depth;
}

//获取最近的关键帧
//void SLAM::checkNearbyLoops(vector<FRAME> &frames, FRAME &curframe, g2o::SparseOptimizer &opti)
//{
//    // 就是把currFrame和 frames里末尾几个测一遍
//    if ( frames.size() <= 5 )
//    {
//        // no enough keyframes, check everyone
//        for (size_t i=0; i<frames.size(); i++)
//        {
//            checkKeyframes( frames[i], curframe, opti, true );
//        }
//    }
//    else
//    {
//        // check the nearest ones
//        for (size_t i = frames.size()-5; i<frames.size(); i++)
//        {
//            checkKeyframes( frames[i], curframe, opti, true );
//        }
//    }
//}

//随机获取关键帧
//void SLAM::checkRandomLoops(vector<FRAME> &frames, FRAME &curframe, g2o::SparseOptimizer &opti)
//{
//    srand( (unsigned int) time(NULL) );
//    // 随机取一些帧进行检测
//
//    if ( frames.size() <= 5 )
//    {
//        // no enough keyframes, check everyone
//        for (size_t i=0; i<frames.size(); i++)
//        {
//            checkKeyframes( frames[i], curframe, opti, true );
//        }
//    }
//    else
//    {
//        // randomly check loops
//        for (int i=0; i<5; i++)
//        {
//            int index = rand()%frames.size();
//            checkKeyframes( frames[index], curframe, opti, true );
//        }
//    }
//}

//判断是否为关键帧
//CHECK_RESULT SLAM::checkKeyframes(FRAME &f1, FRAME &f2, g2o::SparseOptimizer &opti, bool is_loops)
//{
//    static int min_inliers = 5;
//    static double max_norm = 0.2;
//    static double keyframe_threshold = 0.1;
//    static double max_norm_lp = 2.0;
//    // 比较f1 和 f2
//    RESULT_OF_PNP result = slam_base->EstimateMotion( f1, f2, camera );
//
//    // 判断inliers个数，inliers不够，放弃该帧
//    if ( result.inliers < min_inliers )
//        return NOT_MATCHED;
//
//    // 计算运动范围是否太大，运动范围太大，放弃该帧
//    double norm = slam_base->NormalTransform(result.rvec, result.tvec);
//    if (!is_loops)//is_loop用来判断是否是进入回环检测，如果不进入回环检测就是判断前后两帧的距离，如果进入回环检测，这是判断关键帧之间的距离
//    {
//        if ( norm >= max_norm )
//            return TOO_FAR_AWAY;
//    }
//    else
//    {
//        if ( norm >= max_norm_lp)
//            return TOO_FAR_AWAY;
//    }
//
//    // 计算运动范围是否太小，运动范围太小，放弃该帧
//    if ( norm <= keyframe_threshold )
//        return TOO_CLOSE;
//
//    // 前面没有return掉，就会在优化里面添加边
//
//    // 向g2o中增加这个顶点与上一帧联系的边
//    // 顶点部分
//    // 顶点只需设定id即可
//    if (!is_loops)//如果不是进入回环检测的话则添加当前帧，如果进入的话则不添加因为之前已经添加过此帧了
//    {
//        g2o::VertexSE3 *v = new g2o::VertexSE3();
//        v->setId( f2.frame_id );
//        v->setEstimate( Eigen::Isometry3d::Identity() );
//        opti.addVertex(v);
//    }
//    // 边部分
//    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
//    // 连接此边的两个顶点id
//    edge->setVertex( 0, opti.vertex(f1.frame_id ));
//    edge->setVertex( 1, opti.vertex(f2.frame_id ));
//    edge->setRobustKernel( new g2o::RobustKernelHuber() );
//    // 信息矩阵
//    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
//    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
//    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
//    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
//    information(0,0) = information(1,1) = information(2,2) = 100;
//    information(3,3) = information(4,4) = information(5,5) = 100;
//    // 也可以将角度设大一些，表示对角度的估计更加准确
//    edge->setInformation( information );
//    // 边的估计即是pnp求解之结果
//    Eigen::Isometry3d T = slam_base->RotVector2HomoMatrix( result.rvec, result.tvec );
//    edge->setMeasurement( T.inverse() );
//    // 将此边加入图中
//    opti.addEdge(edge);
//    return KEYFRAME;
//}


//主程序
void SLAM::Run()
{
    //用来计时
    clock_t time_a,time_b,time_c,time_d,time_e;

    //用来查看图像和点云
    pcl::visualization::CloudViewer viewer("viewer");
    namedWindow("viewer");

    //使用的一些变量
    FRAME curframe;
    FRAME preframe;
    vector<FRAME> keyframes;
    Mat t_global,R_global;
    bool first_flag = true;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //初始化第一帧，生成第一帧的点云
    preframe.image = GetImage(1);
    preframe.depth = GetDepth(1);
    slam_base->ComputeKeyPointAndDescriptor(preframe);
    slam_base->CreatCloud(preframe.depth,preframe.image,camera,cloud);

//    //------------------------初始化求解器-------------------------------
////    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > SlamBlockSolver;
////    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;
////
////    SlamLinearSolver* linearSolver = new SlamLinearSolver();
////    linearSolver->setBlockOrdering( false );
////    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
////    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
////    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
////    globalOptimizer.setAlgorithm( solver );
////    // 不要输出调试信息
////    globalOptimizer.setVerbose( false );
//
//    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > SlamBlockSolver;  // pose 维度为 6, landmark 维度为 3
//    unique_ptr<SlamBlockSolver::LinearSolverType> SlamLinearSolver ( new g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>());
//    unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver (move(SlamLinearSolver)));// 矩阵块求解器
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (move(blockSolver));
//    g2o::SparseOptimizer globalOptimizer;
//    globalOptimizer.setAlgorithm ( solver );
//    globalOptimizer.setVerbose( false );
//
//    // 向globalOptimizer增加第一个顶点
//    g2o::VertexSE3* v = new g2o::VertexSE3();
//    v->setId( 1 );
//    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
//    v->setFixed( true ); //第一个顶点固定，不用优化
//    globalOptimizer.addVertex( v );
//    //-----------------------------------------------------------------

    //放入第一关键帧
    keyframes.push_back(preframe);

    for(int i = 1; i<782; i++)
    {
        cout << "the frame is " << i << endl;

        time_a = clock();

        //计算新帧的特征点和描述子
        curframe.image = GetImage(i);
        curframe.depth = GetDepth(i);
        slam_base->ComputeKeyPointAndDescriptor(curframe);

        time_b = clock();

        //计算旧帧相对于新帧的变换,由于是拿旧帧作为3d点，新帧作为2d点，因此求出来的变换矩阵的作用是：旧帧的点云乘以变化矩阵能够变化到新的点云的坐标系下
        //RESULT_OF_PNP result = slam_base->EstimateMotion(preframe,curframe,camera);

        //计算新帧相对于新帧的变换
        RESULT_OF_PNP result = slam_base->EstimateMotion(curframe, preframe, camera);
        Mat R, t;
        Rodrigues(result.rvec, R);
        t = result.tvec.clone();
        if (first_flag)
        {
            R_global = R.clone();
            t_global = t.clone();
            first_flag = false;
        }
        else
        {
            t_global = t_global + (R_global*t);
            R_global = R*R_global;
        }

        cout<<R<<endl<<t<<endl;

        //更新旧帧
        preframe.image = curframe.image.clone();
        preframe.depth = curframe.depth.clone();
        preframe.descriptor = curframe.descriptor.clone();
        preframe.keypoint.assign(curframe.keypoint.begin(),curframe.keypoint.end());

        //如果匹配点过少就舍弃此帧
        if(result.inliers < 5)
            continue;

        time_c = clock();

        //拼接点云
        Eigen::Isometry3d T = slam_base->RotMatrix2HomoMatrix(R_global,t_global);
        cloud = slam_base->JoinPointCloud(cloud,curframe,T,camera);

        time_d = clock();

        //显示图像和点云
        imshow("viewer",curframe.image);
        viewer.showCloud(cloud);

        time_e = clock();

        waitKey(1);

        cout<<(double)(time_b-time_a)/CLOCKS_PER_SEC<<" "
            <<(double)(time_c-time_b)/CLOCKS_PER_SEC<<" "
            <<(double)(time_d-time_c)/CLOCKS_PER_SEC<<" "
            <<(double)(time_e-time_d)/CLOCKS_PER_SEC<<endl<<endl;
    }
}

int main(int argc, char** argv)
{
    SLAM slam;
    slam.Run();
    return 0;
}