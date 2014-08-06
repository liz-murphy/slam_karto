/*
 *
 */

#include <open_karto/Karto.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/factory.h>

#include <g2o/types/slam2d/types_slam2d.h>
#include <slam_karto/g2o/vertigo_maxmix_solver.h>
#include <stdlib.h>

G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(vertigo)

void VertigoMaxMixSolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.15;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  visualization_msgs::Marker loop_edge;
  loop_edge.header.frame_id = "map";
  loop_edge.header.stamp = ros::Time::now();
  loop_edge.action = visualization_msgs::Marker::ADD;
  loop_edge.ns = "karto";
  loop_edge.id = 0;
  loop_edge.type = visualization_msgs::Marker::LINE_STRIP;
  loop_edge.scale.x = 0.1;
  loop_edge.scale.y = 0.1;
  loop_edge.scale.z = 0.1;
  loop_edge.color.a = 1.0;
  loop_edge.color.r = 1.0;
  loop_edge.color.g = 0.0;
  loop_edge.color.b = 1.0;

  int loop_id = 0;
  int id = 0;
  m.action = visualization_msgs::Marker::ADD;

  std::set<int> vertex_ids;
  for(g2o::SparseOptimizer::EdgeSet::iterator edge_it = optimizer_->edges().begin(); edge_it != optimizer_->edges().end(); ++edge_it)
  {
    EdgeSE2MaxMixture* edge_maxmix = dynamic_cast<EdgeSE2MaxMixture*>(*edge_it);

    int id1, id2;
    g2o::VertexSE2* v1, *v2;

    bool is_loop_edge;
    double alpha;

    if(edge_maxmix != NULL)
    {
      // Switchable loop closure
      v1 = dynamic_cast<g2o::VertexSE2 *>(edge_maxmix->vertices()[0]);
      v2 = dynamic_cast<g2o::VertexSE2 *>(edge_maxmix->vertices()[1]);
      loop_edge.color.a = 1.0;
      if(edge_maxmix->nullHypothesisMoreLikely) // switched off
      {
        loop_edge.color.a = 0.25;
      }
      is_loop_edge = true;
    }
    else
    {
      g2o::EdgeSE2* edge_odo = dynamic_cast<g2o::EdgeSE2*>(*edge_it);
      if(edge_odo != NULL)
      {
        v1 = dynamic_cast<g2o::VertexSE2 *>(edge_odo->vertices()[0]);
        v2 = dynamic_cast<g2o::VertexSE2 *>(edge_odo->vertices()[1]);
        is_loop_edge = false;
      }
    }
    geometry_msgs::Point p1, p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

    if(is_loop_edge)
    {
      loop_edge.points.clear();
      loop_edge.points.push_back(p1);
      loop_edge.points.push_back(p2);
      loop_edge.id = id;
      marray.markers.push_back(visualization_msgs::Marker(loop_edge));
    }
    else
    {
      edge.points.clear();
      edge.points.push_back(p1);
      edge.points.push_back(p2);
      edge.id = id;
      marray.markers.push_back(visualization_msgs::Marker(edge));
    }
    id++;

    // Check the vertices exist, if not add
    if( vertex_ids.find(v1->id()) == vertex_ids.end() )
    {
      // Add the vertex to the marker array 
      m.id = id;
      m.pose.position.x = p1.x;
      m.pose.position.y = p1.y;
      vertex_ids.insert(v1->id());
      marray.markers.push_back(visualization_msgs::Marker(m));
      id++;
    }
    // Check the vertices exist, if not add
    if( vertex_ids.find(v2->id()) == vertex_ids.end() )
    {
      // Add the vertex to the marker array 
      m.id = id;
      m.pose.position.x = p2.x;
      m.pose.position.y = p2.y;
      vertex_ids.insert(id2);
      marray.markers.push_back(visualization_msgs::Marker(m));
      id++;
    }
  }
}

void VertigoMaxMixSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
  karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
  if(abs(pSource->GetUniqueId() - pTarget->GetUniqueId()) < 20)
  {
    karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

    karto::Pose2 diff = pLinkInfo->GetPoseDifference();
    g2o::SE2 motion(diff.GetX(), diff.GetY(), diff.GetHeading());
   
    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
    Eigen::Matrix<double,3,3> m;
    m(0,0) = precisionMatrix(0,0);
    m(0,1) = m(1,0) = precisionMatrix(0,1);
    m(0,2) = m(2,0) = precisionMatrix(0,2);
    m(1,1) = precisionMatrix(1,1);
    m(1,2) = m(2,1) = precisionMatrix(1,2);
    m(2,2) = precisionMatrix(2,2);
    
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = optimizer_->vertices().find(pSource->GetUniqueId())->second;
    edge->vertices()[1] = optimizer_->vertices().find(pTarget->GetUniqueId())->second;
    
    edge->setMeasurement(motion);
    edge->setInformation(m);

    optimizer_->addEdge(edge);
  }
  else
  {
    std::cout << "Vertigo: Creating maxmix edge\n";

    // Create a switch Vertex
    EdgeSE2MaxMixture* edge_loop = new EdgeSE2MaxMixture();
    edge_loop->vertices()[0] = optimizer_->vertex(pSource->GetUniqueId());
    edge_loop->vertices()[1] = optimizer_->vertex(pTarget->GetUniqueId());

    karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
    karto::Pose2 diff = pLinkInfo->GetPoseDifference();
    g2o::SE2 motion(diff.GetX(), diff.GetY(), diff.GetHeading());

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
    Eigen::Matrix<double,3,3> m;
    m(0,0) = precisionMatrix(0,0);
    m(0,1) = m(1,0) = precisionMatrix(0,1);
    m(0,2) = m(2,0) = precisionMatrix(0,2);
    m(1,1) = precisionMatrix(1,1);
    m(1,2) = m(2,1) = precisionMatrix(1,2);
    m(2,2) = precisionMatrix(2,2);

    edge_loop->setMeasurement(motion);
    edge_loop->setInformation(m);

    edge_loop->weight = 1e-5;
    edge_loop->information_constraint = m;
    edge_loop->nu_constraint = 1.0/sqrt(m.inverse().determinant());
    edge_loop->information_nullHypothesis = m*edge_loop->weight;
    edge_loop->nu_nullHypothesis = 1.0/sqrt(edge_loop->information_nullHypothesis.inverse().determinant());

    std::cout << "Measurement: " << motion[0] << "," << motion[1] << "," << motion[2] << "\n";
    std::cout << "Information: " << m << "\n\n";
    optimizer_->addEdge(edge_loop);
  }

}

