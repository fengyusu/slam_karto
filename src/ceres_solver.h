//
// Created by fengyu on 18-11-15.
//

#ifndef KARTO_CERES_SOLVER_H
#define KARTO_CERES_SOLVER_H

#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_karto/Mapper.h"



class Pose2d {
public:

    Pose2d(double x, double y, double yaw_radians):
          x_(x),y_(y),yaw_radians_(yaw_radians){};

    double x_;
    double y_;
    double yaw_radians_;
};

class Constraint2d {
public:

    int source_id_;
    int target_id_;

    Eigen::Vector3d measurement_;
    Eigen::Matrix3d information_;
};

class CeresSolver : public karto::ScanSolver{
public:
    CeresSolver();
    virtual ~CeresSolver();

    virtual void Clear();
    virtual void Compute();
    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    void getGraph(std::vector<float> &g);

private:
    std::map<int, Pose2d *> user_pose_;
    std::vector<Constraint2d *> user_constraint_;

    karto::ScanSolver::IdPoseVector corrections;

};


#endif //KARTO_CERES_SOLVER_H

