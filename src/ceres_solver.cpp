//
// Created by fengyu on 18-11-15.
//
#include <open_karto/Karto.h>
#include "ceres_solver.h"

#include <ceres/ceres.h>

#include "ros/console.h"


template <typename T>
inline Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
    const T cos_yaw = ceres::cos(yaw_radians);
    const T sin_yaw = ceres::sin(yaw_radians);

    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    return rotation;
}

template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
    T two_pi(2.0 * M_PI);
    return angle_radians -
           two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}



struct PoseGraph2dErrorTerm {
public:
    PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians,
                         const Eigen::Matrix3d& sqrt_information)
            : p_ab_(x_ab, y_ab),
              yaw_ab_radians_(yaw_ab_radians),
              sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                    const T* const x_b, const T* const y_b, const T* const yaw_b,
                    T* residuals_ptr) const {
        const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
        const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

        Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals_ptr);

        residuals_map.template head<2>() = RotationMatrix2D(static_cast<T>(yaw_ab_radians_)).transpose() *
                                          (RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>());
        residuals_map(2) = NormalizeAngle(
                (*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));

        residuals_map = sqrt_information_.template cast<T>() * residuals_map;

        return true;
    }


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Vector2d p_ab_;
    const double yaw_ab_radians_;
    const Eigen::Matrix3d sqrt_information_;
};

void BuildOptimizationProblem(const std::vector<Constraint2d *>& constraints,
                              std::map<int, Pose2d*>* poses,
                              ceres::Problem* problem) {
    CHECK(poses != NULL);
    CHECK(problem != NULL);
    if (constraints.empty()) {
        LOG(INFO) << "No constraints, no problem to optimize.";
        return;
    }

    ceres::LossFunction* loss_function = nullptr;

    for (std::vector<Constraint2d *>::const_iterator constraints_iter =
            constraints.begin();
         constraints_iter != constraints.end(); ++constraints_iter) {
        const Constraint2d* constraint = *constraints_iter;

        std::map<int, Pose2d*>::iterator pose_begin_iter =
                poses->find(constraint->source_id_);
        CHECK(pose_begin_iter != poses->end()) << "Pose with ID: " << constraint->source_id_ << " not found.";

        std::map<int, Pose2d*>::iterator pose_end_iter =
                poses->find(constraint->target_id_);
        CHECK(pose_end_iter != poses->end()) << "Pose with ID: " << constraint->target_id_ << " not found.";

        const Eigen::Matrix3d sqrt_information =
                constraint->information_.llt().matrixL();

        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>
                (new PoseGraph2dErrorTerm(constraint->measurement_[0], constraint->measurement_[1], constraint->measurement_[2], sqrt_information));
        problem->AddResidualBlock(
                cost_function, loss_function,
                &pose_begin_iter->second->x_, &pose_begin_iter->second->y_, &pose_begin_iter->second->yaw_radians_,
                &pose_end_iter->second->x_, &pose_end_iter->second->y_, &pose_end_iter->second->yaw_radians_);

    }

    std::map<int, Pose2d*>::iterator pose_start_iter =
            poses->begin();
    CHECK(pose_start_iter != poses->end()) << "There are no poses.";
    problem->SetParameterBlockConstant(&pose_start_iter->second->x_);
    problem->SetParameterBlockConstant(&pose_start_iter->second->y_);
    problem->SetParameterBlockConstant(&pose_start_iter->second->yaw_radians_);
}


bool SolveOptimizationProblem(ceres::Problem* problem) {
    CHECK(problem != NULL);

    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}


CeresSolver::CeresSolver(){

}

CeresSolver::~CeresSolver(){
    for(auto user_pose : user_pose_){
        delete user_pose.second;
    }
    user_pose_.clear();

    for(auto user_constraint : user_constraint_){
        delete user_constraint;
    }
    user_constraint_.clear();

}

void CeresSolver::Clear()
{
  corrections.clear();

}

const karto::ScanSolver::IdPoseVector& CeresSolver::GetCorrections() const
{
  return corrections;
}

void CeresSolver::Compute()
{
  corrections.clear();

  ROS_INFO("Calling ceres for loop closure");
  ceres::Problem problem;
  BuildOptimizationProblem(user_constraint_, &user_pose_, &problem);
  SolveOptimizationProblem(&problem);
  ROS_INFO("Finished ceres for loop closure");
  
  for(auto user_pose : user_pose_){
    karto::Pose2 pose(user_pose.second->x_, user_pose.second->y_, user_pose.second->yaw_radians_);
    corrections.push_back(std::make_pair(user_pose.first, pose));
  }
}

void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  int pose_id = pVertex->GetObject()->GetUniqueId();
  Pose2d *pose2d = new Pose2d(pose.GetX(), pose.GetY(), pose.GetHeading());
  user_pose_[pose_id] = pose2d;

  ROS_INFO("AddNode %d",pVertex->GetObject()->GetUniqueId());
}

void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
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
  Eigen::Vector3d measurement(diff.GetX(), diff.GetY(), diff.GetHeading());

  Constraint2d *constraint2d = new Constraint2d();
  constraint2d->source_id_ = pSource->GetUniqueId();
  constraint2d->target_id_ = pTarget->GetUniqueId();
  constraint2d->measurement_ = measurement;
  constraint2d->information_ = m;
  user_constraint_.push_back(constraint2d);

  ROS_INFO("AddConstraint %d  %d", pSource->GetUniqueId(), pTarget->GetUniqueId());

}

void CeresSolver::getGraph(std::vector<float> &g) { 
    for(auto user_constraint : user_constraint_)
    {
        const Pose2d* source_pose = user_pose_[user_constraint->source_id_];
        const Pose2d* target_pose = user_pose_[user_constraint->target_id_];
        g.push_back(source_pose->x_);
        g.push_back(source_pose->y_);
        g.push_back(target_pose->x_);
        g.push_back(target_pose->y_);
    }
}
