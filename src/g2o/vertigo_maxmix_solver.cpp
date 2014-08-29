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

bool VertigoMaxMixSolver::getEdgeStatus(g2o::OptimizableGraph::Edge* edge)
{
  EdgeSE2MaxMixture* edge_maxmix = dynamic_cast<EdgeSE2MaxMixture*>(edge);
 
  bool status = true;

  if(edge_maxmix != NULL)
  {
    // Switchable loop closure
    if(edge_maxmix->nullHypothesisMoreLikely) // switched off
    {
      status = false;
    }
  }
  return status;
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

