#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include <slam_karto/slam_karto_solver.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>

namespace g2o
{
  class EdgeSE2;
  class VertexSE2;
  class SparseOptimizer;
  class RawLaser;
  class RobotLaser;
  class LaserParameters;
}

typedef std::pair<int,int> edge_pair_t;
typedef std::map<edge_pair_t,bool> loop_status_t;
typedef std::map<edge_pair_t,visualization_msgs::InteractiveMarker> loop_marker_t;
typedef std::map<edge_pair_t,g2o::EdgeSE2> edge_data_t;
typedef std::map<edge_pair_t,g2o::EdgeSE2*> edge_data_pt_t;

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
  bool use_interactive_switches_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  visualization_msgs::Marker MakeSwitch(visualization_msgs::InteractiveMarker &msg, geometry_msgs::Point &p1, geometry_msgs::Point &p2, bool status);
  visualization_msgs::Marker MakeEdge(visualization_msgs::InteractiveMarker &msg, geometry_msgs::Point &p1, geometry_msgs::Point &p2, bool status);
  virtual bool getEdgeStatus(g2o::EdgeSE2* edge);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void getLoopClosures(visualization_msgs::MarkerArray &marray); // each of the variants handles rendering loop closures differently
  void getOdometryGraph(visualization_msgs::MarkerArray &marray);
  //std::vector<g2o::EdgeSE2*> loop_closure_edges_;
  bool optimized_since_last_visualization_;
  //visualization_msgs::MarkerArray loop_closure_array_;
  loop_status_t loop_closure_status_map_;
  loop_marker_t loop_closure_markers_;
  edge_data_pt_t active_edges_;
  edge_data_t inactive_edges_;
};

#endif // KARTO_G2OSOLVER_H

