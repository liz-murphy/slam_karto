#ifndef KARTO_VERTIGO_SWITCHABLE_SOLVER_H
#define KARTO_VERTIGO_SWITCHABLE_SOLVER_H

#include "g2o_solver.h" 
#include <vertigo/g2o/edge_se2Switchable.h>
#include <vertigo/g2o/edge_switchPrior.h>
#include <vertigo/g2o/vertex_switchLinear.h>
#include <visualization_msgs/MarkerArray.h>

// Vertigo class definitions
class VertexSwitchLinear;
class EdgeSE2Switchable;
class EdgeSwitchPrior;

class VertigoSwitchableSolver : public G2OSolver 
{
  public:
    VertigoSwitchableSolver(); // Nasty hack
    void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);
    virtual bool getEdgeStatus(g2o::OptimizableGraph::Edge* edge);
    void Compute();
    virtual bool turnEdgeOn(g2o::OptimizableGraph::Edge* e);
    virtual bool turnEdgeOff(g2o::OptimizableGraph::Edge* e);
  private:
    int _switch_id;
    double Ksi_; // covariance on switch constraint (DEFAULT: 1, 0.1 works in real datasets? (MM vs RRR vs Switchable paper)
};

#endif // KARTO_VERTIGO_SWITCHABLE_SOLVER_H

