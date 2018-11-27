//
// Created by fengyu on 18-11-15.
//

#ifndef KARTO_G2O_SOLVER_H
#define KARTO_G2O_SOLVER_H

#include <vector>
#include <map>

#include "g2o_karto/se2.h"
#include "open_karto/Mapper.h"

using namespace g2o;
using namespace g2o::tutorial;


typedef struct vertex
{
    Eigen::Vector3d pose;
}Vertex;

typedef struct edge
{
    int source_id,target_id;
    SE2 measurement;
    Eigen::Matrix3d info_matrix;
}UserEdge;





class G2oSolver : public karto::ScanSolver{
public:
    G2oSolver();
    virtual ~G2oSolver();

    virtual void Clear();
    virtual void Compute();
    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    void getGraph(std::vector<float> &g);

private:
    std::vector<SE2 *> user_vertex_;
    std::vector<UserEdge *> user_edge_;

    karto::ScanSolver::IdPoseVector corrections;

};


#endif //KARTO_G2O_SOLVER_H

