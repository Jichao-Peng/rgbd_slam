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
void SLAM::CheckNearbyLoops(vector<FRAME> &frames, FRAME &curframe, g2o::SparseOptimizer &opti)
{
    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= 5 )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            CheckKeyframes( frames[i], curframe, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-5; i<frames.size(); i++)
        {
            CheckKeyframes( frames[i], curframe, opti, true );
        }
    }
}

//随机获取关键帧
void SLAM::CheckRandomLoops(vector<FRAME> &frames, FRAME &curframe, g2o::SparseOptimizer &opti)
{
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测

    if ( frames.size() <= 5 )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            CheckKeyframes( frames[i], curframe, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<5; i++)
        {
            int index = rand()%frames.size();
            CheckKeyframes( frames[index], curframe, opti, true );
        }
    }
}

//判断是否为关键帧
CHECK_RESULT SLAM::CheckKeyframes(FRAME &frame1, FRAME &frame2, g2o::SparseOptimizer &opti, bool is_loops)
{
    static int min_inliers = 2;
    static double max_norm = 0.2;
    static double keyframe_threshold = 0.1;
    static double max_norm_lp = 2.0;
    // 比较frame1 和 frame2
    cout<<"matching frame "<<frame1.frame_id<<" and "<<frame2.frame_id<<" "<<endl;
    RESULT_OF_PNP result = slam_base->EstimateMotion( frame1, frame2, camera );//求的是关键帧到当前帧的齐次变换矩阵

    // 判断inliers个数，inliers不够，放弃该帧
    cout<<"inliers number is "<<result.inliers<<endl;
    if ( result.inliers < min_inliers )
        return NOT_MATCHED;

    // 计算运动范围是否太大，运动范围太大，放弃该帧
    double norm = slam_base->NormalTransform(result.rvec, result.tvec);
    if (!is_loops)//is_loop用来判断是否是进入回环检测，如果不进入回环检测就是判断前后两帧的距离，如果进入回环检测，这是判断关键帧之间的距离
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    // 计算运动范围是否太小，运动范围太小，放弃该帧
    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;

    // 前面没有return掉，就会在优化里面添加边，因此只有符合要求的关键帧才会进行BA优化


    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (!is_loops)//如果不是进入回环检测的话则添加当前帧，如果进入的话则不添加因为之前已经添加过此帧了
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( frame2.frame_id );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->setVertex( 0, opti.vertex(frame1.frame_id ));//前一关键帧
    edge->setVertex( 1, opti.vertex(frame2.frame_id ));//当前帧
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = slam_base->RotVector2HomoMatrix( result.rvec, result.tvec );
    edge->setMeasurement( T.inverse() );//注意！！！！！求出来的T是由当前帧到前一帧的齐次变换矩阵 TODO:暂定是由尾指向头
    // 将此边加入图中
    opti.addEdge(edge);
    return KEYFRAME;
}


//主程序
void SLAM::Run()
{
    //用来查看图像和点云
    pcl::visualization::CloudViewer viewer("viewer");
    namedWindow("viewer");

    //使用的一些变量
    int start_index = 50;
    FRAME firstframe;
    vector<FRAME> keyframes;
    Mat t_global,R_global;
    bool first_flag = true;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //初始化第一帧，生成第一帧的点云
    firstframe.image = GetImage(start_index);
    firstframe.depth = GetDepth(start_index);
    firstframe.frame_id = start_index;
    slam_base->ComputeKeyPointAndDescriptor(firstframe);
    slam_base->CreatCloud(firstframe.depth,firstframe.image,camera,cloud);

    //------------------------初始化求解器-------------------------------
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > SlamBlockSolver;  // pose 维度为 6, landmark 维度为 3
    unique_ptr<SlamBlockSolver::LinearSolverType> SlamLinearSolver ( new g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>());
    unique_ptr<SlamBlockSolver> block_solver ( new SlamBlockSolver (move(SlamLinearSolver)));// 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (move(block_solver));
    g2o::SparseOptimizer global_optimizer;
    global_optimizer.setAlgorithm ( solver );
    global_optimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( start_index );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化.
    global_optimizer.addVertex( v );
    //-----------------------------------------------------------------

    //放入第一关键帧
    keyframes.push_back(firstframe);

    for(int i = start_index+1; i<782; i++)
    {
        FRAME curframe;
        cout<<"the frame is "<<i<<endl;

        curframe.image = GetImage(i);
        curframe.depth = GetDepth(i);
        curframe.frame_id = i;

        slam_base->ComputeKeyPointAndDescriptor(curframe);
        CHECK_RESULT result = CheckKeyframes(keyframes.back(),curframe,global_optimizer);//求的是关键帧到当前帧的齐次变换矩阵
        switch(result)
        {
            case NOT_MATCHED:
                //没匹配上，直接跳过
                cout<<"not enough inliers"<<endl;
                break;
            case TOO_FAR_AWAY:
                // 太近了，也直接跳
                cout<<"too far away, may be an error"<<endl;
                break;
            case TOO_CLOSE:
                // 太远了，可能出错了
                cout<<"too close, not a keyframe"<<endl;
                break;
            case KEYFRAME:
                cout<<"bingo------------------------------------> this is a new keyframe"<<endl;
                CheckNearbyLoops( keyframes, curframe, global_optimizer );
                CheckRandomLoops( keyframes, curframe, global_optimizer );
                keyframes.push_back(curframe);

                //每添加一个关键帧就进行一次BA优化
                if(keyframes.size()>2)
                {
                    cout<<"start BA"<<endl;
                    global_optimizer.initializeOptimization();
                    global_optimizer.optimize(100); //可以指定优化步数
                    cout<<"finish BA"<<endl;

                    g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(global_optimizer.vertex(
                            curframe.frame_id));
                    Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
                    cloud = slam_base->JoinPointCloud(cloud, curframe, pose, camera);
                }
        }
        imshow("viewer",curframe.image);
        viewer.showCloud(cloud);
        waitKey(1);
        cout<<endl;
    }

}

int main(int argc, char** argv)
{
    SLAM slam;
    slam.Run();
    return 0;
}