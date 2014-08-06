/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <open_karto/Karto.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/factory.h>

#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/data/types_data.h>
#include <slam_karto/g2o/types/data/robot_laser_sclam.h>

#include <slam_karto/g2o/g2o_solver.h>
#include "ros/console.h"

#include <slam_karto/localized_range_scan_stamped.h>

G2O_USE_TYPE_GROUP(data)
G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(calibration)

G2OSolver::G2OSolver()
{
  calibration_debug_ = false;
  optimizer_ = new g2o::SparseOptimizer();

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solverGauss   = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  // OptimizationAlgorithmLevenberg* solverLevenberg = new OptimizationAlgorithmLevenberg(blockSolver);
  optimizer_->setAlgorithm(solverGauss);
}

G2OSolver::~G2OSolver()
{
  delete optimizer_;
}

void G2OSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector& G2OSolver::GetCorrections() const
{
  return corrections_;
}

void G2OSolver::Compute()
{
  std::cout << "G2OSolver::Compute(): saving graph..." << std::flush;
  optimizer_->save("before_optimization.g2o");
  std::cout << "done." << std::endl;
  std::cout << "G2OSolver::Compute(): running optimizer..." << std::flush;
  corrections_.clear();
  //typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;

  optimizer_->initializeOptimization();
  optimizer_->optimize(50);

  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    const g2o::SE2& estimate = vertices_[i]->estimate();
    karto::Pose2 pose(estimate.translation().x(), estimate.translation().y(), estimate.rotation().angle());
    corrections_.push_back(std::make_pair(vertices_[i]->id(), pose));
   
    // Update the user data
    /*if(vertices_[i]->userData() != NULL)
    {
    }*/
  }
  optimizer_->save("after_optimization.g2o");
        
  /*
  g2o::OptimizableGraph::VertexContainer points;
  for (
      g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer_->vertices().begin(); 
      it != optimizer_->vertices().end(); ++it) 
  {
    g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
    const g2o::SE2& estimate = v->estimate();
    karto::Pose2 pose(estimate.translation().x(), estimate.translation().y(), estimate.rotation().angle());
  }
  */
}

void G2OSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  //Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
  //m_Spa.addNode(vector, pVertex->GetObject()->GetUniqueId());
  g2o::VertexSE2* vertex = new g2o::VertexSE2();
  vertex->setId(pVertex->GetObject()->GetUniqueId());
  g2o::SE2 p(pose.GetX(), pose.GetY(), pose.GetHeading());
  vertex->setEstimate(p);
  // fix first vertex
  if (vertices_.size() == 0)
  {
    vertex->setFixed(true);
  }
 
  if(calibration_debug_)
  {
    g2o::RobotLaserSCLAM* rl = new g2o::RobotLaserSCLAM();
    g2o::LaserParameters *lp = new g2o::LaserParameters(1081,-2.2689,0.004363,60);
    std::vector<double> readings(pVertex->GetObject()->GetRangeReadings(), pVertex->GetObject()->GetRangeReadings()+pVertex->GetObject()->GetNumberOfRangeReadings());
    rl->setRanges(readings);
    readings.clear();
    rl->setRemissions(readings);
    karto::Pose2 odomPose = pVertex->GetObject()->GetOdometricPose();
    //karto::Pose2 odomPose = pVertex->GetObject()->GetCorrectedPose();
    g2o::SE2 pOdom(odomPose.GetX(), odomPose.GetY(), odomPose.GetHeading());
    rl->setOdomPose(pOdom);
    rl->setLaserParams(*lp);
    double ts =  static_cast<karto::LocalizedRangeScanStamped *>(pVertex->GetObject())->getTimestamp();
    rl->setTimestamp((double)ts);
    vertex->setUserData(rl);
  }

  optimizer_->addVertex(vertex);
  //TODO Memory management of vertices and edges?
  vertices_.push_back(vertex);
}

void G2OSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
  karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  //Eigen::Vector3d mean(diff.GetX(), diff.GetY(), diff.GetHeading());
  g2o::SE2 motion(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double,3,3> m;
  m(0,0) = precisionMatrix(0,0);
  m(0,1) = m(1,0) = precisionMatrix(0,1);
  m(0,2) = m(2,0) = precisionMatrix(0,2);
  m(1,1) = precisionMatrix(1,1);
  m(1,2) = m(2,1) = precisionMatrix(1,2);
  m(2,2) = precisionMatrix(2,2);

  //m_Spa.addConstraint(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);

  g2o::EdgeSE2* edge = new g2o::EdgeSE2();
  edge->vertices()[0] = optimizer_->vertices().find(pSource->GetUniqueId())->second;
  edge->vertices()[1] = optimizer_->vertices().find(pTarget->GetUniqueId())->second;

  edge->setMeasurement(motion);
  edge->setInformation(m);

  optimizer_->addEdge(edge);
}

void G2OSolver::getGraph(std::vector<float> &g)
{
  // Return graph as series of vertices V1_x, V1_y, V2_x, V2_y ...
  for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); ++it)
  {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2 *>(it->second);

    g2o::SE2 pose = v->estimate();
    g.push_back(pose[0]);
    g.push_back(pose[1]);
  }
}

void G2OSolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
{
  visualization_msgs::Marker m;
  m.header.frame_id = map_frame_id_;
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
  edge.header.frame_id = map_frame_id_;
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

  int id = 0;
  m.action = visualization_msgs::Marker::ADD;

  std::set<int> vertex_ids;
  for(g2o::SparseOptimizer::EdgeSet::iterator edge_it = optimizer_->edges().begin(); edge_it != optimizer_->edges().       end(); ++edge_it)
  {
    int id1, id2;
    g2o::VertexSE2* v1, *v2;

    // Switchable loop closure
    v1 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[0]);
    v2 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[1]);
    
    geometry_msgs::Point p1, p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

   edge.points.clear();
   edge.points.push_back(p1);
   edge.points.push_back(p2);
   edge.id = id;
   marray.markers.push_back(visualization_msgs::Marker(edge));
   id++;
     
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
