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
  VertigoSwitchableSolver():_switch_id(10000){}; // Nasty hack
  void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);
  void publishGraphVisualization(visualization_msgs::MarkerArray &marray);
  void Compute();
  private:
    int _switch_id;
};

#endif // KARTO_VERTIGO_SWITCHABLE_SOLVER_H

