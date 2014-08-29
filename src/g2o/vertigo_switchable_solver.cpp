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
#include <slam_karto/g2o/vertigo_switchable_solver.h>
#include <stdlib.h>

#include <slam_karto/g2o/types/data/robot_laser_sclam.h>

#include <ros/ros.h>
#include <ros/console.h>

G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(vertigo)

VertigoSwitchableSolver::VertigoSwitchableSolver() : _switch_id(10000)
{
  // Read in Ksi parameter
  ros::NodeHandle nh("~");
  if(!nh.hasParam("Ksi"))
  {
    Ksi_ = 1.0;
    ROS_ERROR("Could not find a Ksi parameter, setting to default 1.0");
  }
  else
  {
    nh.getParam("Ksi", Ksi_);
    ROS_INFO("Switchable covariance is %f", Ksi_);
  }

}

bool VertigoSwitchableSolver::getEdgeStatus(g2o::OptimizableGraph::Edge* edge)
{
  EdgeSE2Switchable* edge_switch = dynamic_cast<EdgeSE2Switchable*>(edge);
 
  bool status = true;

  if(edge_switch != NULL)
  {
      VertexSwitchLinear *v3 = dynamic_cast<VertexSwitchLinear *>(edge_switch->vertices()[2]);
      if(v3->x() < 0.5) // switched off
      {
        status = false;
      }
  }
  return status;
}

void VertigoSwitchableSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
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
    std::cout << "Vertigo: Creating switchable edge\n";
    // Create a switch Vertex
    VertexSwitchLinear* vertex = new VertexSwitchLinear();
    vertex->setId(_switch_id);
    double init_value = 1.0;
    vertex->setEstimate(init_value);
    optimizer_->addVertex(vertex);

    // Create a Unary Edge
    EdgeSwitchPrior *edge_prior = new EdgeSwitchPrior();
    edge_prior->vertices()[0] = optimizer_->vertex(_switch_id);
    edge_prior->setMeasurement(1.0);
    //edge_prior->setMeasurement(0.0);
    edge_prior->setInformation((1./Ksi_)*Eigen::MatrixXd::Identity(1,1));
    optimizer_->addEdge(edge_prior);

    // Create a switchable edge
    EdgeSE2Switchable *edge_loop = new EdgeSE2Switchable();
    edge_loop->vertices()[0] = optimizer_->vertex(pSource->GetUniqueId());
    edge_loop->vertices()[1] = optimizer_->vertex(pTarget->GetUniqueId());
    edge_loop->vertices()[2] = optimizer_->vertex(_switch_id++);

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
    //m = Eigen::MatrixXd::Identity(3,3);
    edge_loop->setMeasurement(motion);
    edge_loop->setInformation(m);
    std::cout << "Measurement: " << motion[0] << "," << motion[1] << "," << motion[2] << "\n";
    std::cout << "Information: " << m << "\n\n";
    optimizer_->addEdge(edge_loop);
  }

}

void VertigoSwitchableSolver::Compute()
{
  std::cout << "G2OSolver::Compute(): saving graph..." << std::flush;
  optimizer_->save("before_optimization.g2o");
  std::cout << "done." << std::endl;
  std::cout << "G2OSolver::Compute(): running optimizer..." << std::flush;
  corrections_.clear();
  optimizer_->initializeOptimization(0);
  optimizer_->optimize(optimization_iterations_,online_optimization_);
  
  for(size_t i = 0; i < vertices_.size(); ++i)
  {
    const g2o::SE2& estimate = vertices_[i]->estimate();
    karto::Pose2 pose(estimate.translation().x(), estimate.translation().y(), estimate.rotation().angle());
    corrections_.push_back(std::make_pair(vertices_[i]->id(), pose));
 
    if(calibration_debug_)
    {
      // Update the user data
      std::cout << "Data type: " << vertices_[i]->userData()->elementType() << "\n";
      g2o::RobotLaserSCLAM* data_ptr = static_cast<g2o::RobotLaserSCLAM *>(vertices_[i]->userData());
      if(data_ptr != NULL)
      {
        data_ptr->setCorrectedPose(vertices_[i]->estimate());
      }
    }
  }
  
  std::cout << "done." << std::endl;
  optimizer_->save("after_optimization.g2o");
}

bool VertigoSwitchableSolver::turnEdgeOn(g2o::OptimizableGraph::Edge* e)
{
  // There are 2 edges associated with each loop closure (ternary and unary)
  // e is the reference to the ternary edge which contains the vertex of the unary edge

  EdgeSE2Switchable* ternary_edge = dynamic_cast<EdgeSE2Switchable *>(e);
  VertexSwitchLinear* switch_vertex = (VertexSwitchLinear*)ternary_edge->vertices()[2];

  for(g2o::HyperGraph::EdgeSet::iterator it = switch_vertex->edges().begin(); it != switch_vertex->edges().end(); ++it)
  {
    ((g2o::OptimizableGraph::Edge*)*it)->setLevel(0);
  }
}

bool VertigoSwitchableSolver::turnEdgeOff(g2o::OptimizableGraph::Edge* e)
{
  EdgeSE2Switchable* ternary_edge = dynamic_cast<EdgeSE2Switchable *>(e);
  VertexSwitchLinear* switch_vertex = (VertexSwitchLinear*)ternary_edge->vertices()[2];

  for(g2o::HyperGraph::EdgeSet::iterator it = switch_vertex->edges().begin(); it != switch_vertex->edges().end(); ++it)
  {
    ((g2o::OptimizableGraph::Edge*)*it)->setLevel(1);
  }
}
