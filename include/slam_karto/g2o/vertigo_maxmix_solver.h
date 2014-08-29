#ifndef KARTO_VERTIGO_MAXMIX_SOLVER_H
#define KARTO_VERTIGO_MAXMIX_SOLVER_H

#include "g2o_solver.h" 
#include <vertigo/g2o/edge_se2MaxMixture.h>
#include <visualization_msgs/MarkerArray.h>

// Vertigo class definitions
class EdgeSE2MaxMixture;

class VertigoMaxMixSolver : public G2OSolver 
{
  public:
  void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);
  virtual bool getEdgeStatus(g2o::OptimizableGraph::Edge* edge);
};

#endif // KARTO_VERTIGO_G2OSOLVER_H

