
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "optimizer.h"


typedef g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType> SlamLinearSolver;


void Optimizer::BundleAdjustmentSE3(vector<Frame> &frames, vector<MapPoint> &mappoints,cv::Mat& K,size_t iterations) {

    g2o::SparseOptimizer optimizer;
    //step1: initialization
    auto linearSolver=g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    auto blockSolver=g2o::make_unique<g2o::BlockSolverX>(move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(move(blockSolver));

    optimizer.setAlgorithm(algorithm);

    //step2: add camera vertex
    int id=0;
    g2o::VertexSE3Expmap* vSE3;
    for(auto it:frames){
        if(it.is_init) {
            vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setId(id);
            vSE3->setEstimate(it.Pose());
            vSE3->setFixed(!it.idx);
            optimizer.addVertex(vSE3);
            id++;
        }
    }
    int nframe=id;

    //step3: add landmarks and edges
    g2o::VertexSBAPointXYZ* vXYZ;
    g2o::EdgeSE3ProjectXYZ* e;
    for(auto it:mappoints){
        if(it.is_valid) {
            vXYZ = new g2o::VertexSBAPointXYZ();
            vXYZ->setId(id);
            vXYZ->setEstimate(it.Postion());
            vXYZ->setFixed(false);
            optimizer.addVertex(vXYZ);
            for (auto itobs:it.mpObservations) {
                size_t fidx = itobs.first;
                size_t kpidx = itobs.second;
                Eigen::Matrix<double, 2, 1> kpun;
                kpun << frames[fidx].mvKeyPointsUn[kpidx].pt.x, frames[fidx].mvKeyPointsUn[kpidx].pt.y;
                e = new g2o::EdgeSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(fidx)));
                e->setMeasurement(kpun);
                e->setInformation(Eigen::Matrix2d::Identity());
                e->fx = K.at<double>(0, 0);
                e->fy = K.at<double>(1, 1);
                e->cx = K.at<double>(0, 2);
                e->cy = K.at<double>(1, 2);
                auto rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(sqrt(5.99));
                optimizer.addEdge(e);
            }
        }
        id++;
    }

    //step4: solve
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(iterations);

    sleep(3);
    //step5: recover result

    for(auto &it:frames){
        if(it.is_init) {
            id = it.idx;
            vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(id));
            it.SetPose(vSE3->estimate());
            cout << "frame " << id << " :" << endl;
            cout << vSE3->estimate() << endl;
        }
    }
    for(auto &it:mappoints){
        if(it.is_valid) {
            id = it.idx + nframe;
            vXYZ = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id));
            it.SetPostion(vXYZ->estimate());
        }
    }


}


//TODO::implement
void Optimizer::BundleAdjustmentSim3(vector<Frame> &frames, vector<MapPoint> &mappoints, cv::Mat &K,size_t iterations) {


}