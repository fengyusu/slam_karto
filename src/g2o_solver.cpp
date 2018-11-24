//
// Created by fengyu on 18-11-15.
//
#include <open_karto/Karto.h>
#include "g2o_solver.h"


#include "g2o_karto/vertex_se2.h"
#include "g2o_karto/vertex_point_xy.h"
#include "g2o_karto/edge_se2.h"
#include "g2o_karto/edge_se2_pointxy.h"
#include "g2o_karto/types_tutorial_slam2d.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "ros/console.h"

using SlamBlockSolver = BlockSolver< BlockSolverTraits<-1, -1> >;
using SlamLinearSolver = LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> ;

SlamLinearSolver* linear_solver_ = nullptr;
SlamBlockSolver* block_solver_ = nullptr;
OptimizationAlgorithmGaussNewton* solver_ = nullptr;
SparseOptimizer optimizer_;

G2oSolver::G2oSolver(){

    linear_solver_ = new SlamLinearSolver();
    linear_solver_->setBlockOrdering(false);
    block_solver_ = new SlamBlockSolver(linear_solver_);
    solver_ = new OptimizationAlgorithmGaussNewton(block_solver_);


    optimizer_.setAlgorithm(solver_);
    optimizer_.setVerbose( true );

}

G2oSolver::~G2oSolver(){
    for(auto vertex_se2 : user_vertex_){
        delete vertex_se2;
    }
    user_vertex_.clear();

    for(auto vertex_se2 : user_edge_){
        delete vertex_se2;
    }
    user_edge_.clear();

    delete linear_solver_;
    delete block_solver_;
    delete solver_;
}





void G2oSolver::Clear()
{
  corrections.clear();

}

const karto::ScanSolver::IdPoseVector& G2oSolver::GetCorrections() const
{
  return corrections;
}

void G2oSolver::Compute()
{
  corrections.clear();

  VertexSE2* first_pose = dynamic_cast<VertexSE2*>(optimizer_.vertex(0));
    first_pose->setFixed(true);


  ROS_INFO("Calling g2o for loop closure");
  optimizer_.initializeOptimization();
    optimizer_.optimize(3);
  ROS_INFO("Finished g2o for loop closure");
  
//   optimizer_.vertices().clear();
    // optimizer_.clear();

    // Factory::destroy();
    // OptimizationAlgorithmFactory::destroy();
    // HyperGraphActionLibrary::destroy();

}

void G2oSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  SE2 *v_se2 = new SE2(pose.GetX(), pose.GetY(), pose.GetHeading());
  user_vertex_.push_back(v_se2);
  VertexSE2* robot_pose =  new VertexSE2;
  robot_pose->setId(pVertex->GetObject()->GetUniqueId());
  robot_pose->setEstimate(*v_se2);
  optimizer_.addVertex(robot_pose);

  ROS_INFO("AddNode %d",pVertex->GetObject()->GetUniqueId());
}

void G2oSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
  karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double,3,3> m;
  m(0,0) = precisionMatrix(0,0);
  m(0,1) = m(1,0) = precisionMatrix(0,1);
  m(0,2) = m(2,0) = precisionMatrix(0,2);
  m(1,1) = precisionMatrix(1,1);
  m(1,2) = m(2,1) = precisionMatrix(1,2);
  m(2,2) = precisionMatrix(2,2);

  EdgeSE2* constraint = new EdgeSE2;
  UserEdge *user_edge = new UserEdge();
  user_edge->source_id = pSource->GetUniqueId();
  user_edge->target_id = pTarget->GetUniqueId();
  user_edge->measurement = {diff.GetX(), diff.GetY(), diff.GetHeading()};
  user_edge->info_matrix = m;
  user_edge_.push_back(user_edge);
  constraint->vertices()[0] = optimizer_.vertex(user_edge->source_id);
  constraint->vertices()[1] = optimizer_.vertex(user_edge->target_id);
  constraint->setMeasurement(user_edge->measurement);
  constraint->setInformation(user_edge->info_matrix);
  optimizer_.addEdge(constraint);

  ROS_INFO("AddConstraint %d  %d", pSource->GetUniqueId(), pTarget->GetUniqueId());

}

void G2oSolver::getGraph(std::vector<float> &g) { 
    for(auto p_edge : user_edge_)
    {
        VertexSE2* source_pose = dynamic_cast<VertexSE2*>(optimizer_.vertex(p_edge->source_id));
        VertexSE2* target_pose = dynamic_cast<VertexSE2*>(optimizer_.vertex(p_edge->target_id));
        g.push_back(source_pose->estimate()[0]);
        g.push_back(source_pose->estimate()[1]);
        g.push_back(target_pose->estimate()[0]);
        g.push_back(target_pose->estimate()[1]);
    }
}
