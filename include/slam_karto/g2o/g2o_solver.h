#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include <slam_karto/slam_karto_solver.h>

namespace g2o
{
  class VertexSE2;
  class SparseOptimizer;
  class RawLaser;
  class RobotLaser;
  class LaserParameters;
}

class G2OSolver : public SlamKartoSolver
{
public:
  G2OSolver();
  virtual ~G2OSolver();

public:
  virtual void Clear();
  virtual void Compute();
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);
  virtual void getGraph(std::vector<float> &g);

  void publishGraphVisualization(visualization_msgs::MarkerArray &marray);
protected:
  karto::ScanSolver::IdPoseVector corrections_;
  g2o::SparseOptimizer* optimizer_;
  std::vector<g2o::VertexSE2*> vertices_;
  bool calibration_debug_;
};

#endif // KARTO_G2OSOLVER_H

