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

#include <slam_karto/localized_range_scan_stamped.h>
#include <tf/tf.h>
#include <boost/lexical_cast.hpp>
#include <cstdlib>

G2O_USE_TYPE_GROUP(data)
G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(calibration)

G2OSolver::G2OSolver()
{
  ros::NodeHandle nh("~");
  if(!nh.getParam("g2o_online_optimization",online_optimization_))
  {
    online_optimization_=false;
    ROS_INFO("Not using online optimization");
  }
  else
    ROS_INFO("Online optimization set to %s", online_optimization_ ? "true" : "false");

  if(!nh.getParam("g2o_num_iterations",optimization_iterations_))
  {
    optimization_iterations_=30;
    ROS_INFO("Using default number of iterations: %d", optimization_iterations_);
  }
  else
    ROS_INFO("Using %d g2o iterations", optimization_iterations_);

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

  server_.reset( new interactive_markers::InteractiveMarkerServer("loop_controls","",false) );
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

  optimizer_->initializeOptimization(0);  // ignore level 1 vertices
  optimizer_->optimize(optimization_iterations_,online_optimization_);

  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    const g2o::SE2& estimate = vertices_[i]->estimate();
    karto::Pose2 pose(estimate.translation().x(), estimate.translation().y(), estimate.rotation().angle());
    corrections_.push_back(std::make_pair(vertices_[i]->id(), pose));
  }
  optimizer_->save("after_optimization.g2o");
  std::cout << "G2OSolver::Compute(): optimization done ..." << std::flush;
}

void G2OSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
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

/* Fill out a visualization_msg to display the odometry graph
 * This function also identifies the loop closure edges and adds them to the
 * loop_closure_edges_ data for further processing
 * */

void G2OSolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
{
  // Vertices are round, red spheres
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

  // Odometry edges are opaque blue line strips 
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

  // Loop edges are purple, opacity depends on backend state
  visualization_msgs::Marker loop_edge;
  loop_edge.header.frame_id = map_frame_id_;
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
 
  int id = (int)marray.markers.size();
  m.action = visualization_msgs::Marker::ADD;

  std::set<int> vertex_ids;
  // HyperGraph Edges
  for(g2o::SparseOptimizer::EdgeSet::iterator edge_it = optimizer_->edges().begin(); edge_it != optimizer_->edges().end(); ++edge_it)
  {
    // Is it a unary edge? Need to skip or else die a bad death
    if( (*edge_it)->vertices().size() < 2 )
      continue;

    int id1, id2;
    g2o::VertexSE2* v1, *v2;

    v1 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[0]);
    v2 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[1]);
   
    geometry_msgs::Point p1, p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

   if( (v2->id() - v1->id()) < 20) // not a loop closure
   {
     edge.points.clear();
     edge.points.push_back(p1);
     edge.points.push_back(p2);
     edge.id = id;
     marray.markers.push_back(visualization_msgs::Marker(edge));
     id++;
   }
   else // it's a loop closure
   {
     g2o::VertexSE2* v1, *v2;
     
     if((*edge_it)->vertices().size() < 2)
       continue;

     v1 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[0]);
     v2 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[1]);
     
     geometry_msgs::Point p1, p2;
     p1.x = v1->estimate()[0];
     p1.y = v1->estimate()[1];
     p2.x = v2->estimate()[0];
     p2.y = v2->estimate()[1];

     if(use_switchable_markers_)
     {
       edge_pair_t current_edge = std::make_pair<int,int>(v1->id(),v2->id());
       loop_status_t::iterator map_iter = loop_closure_status_map_.find(current_edge);
       if( map_iter == loop_closure_status_map_.end()  ) 
       {
         // New loop closure edge, need to create an interactive marker for it
         visualization_msgs::InteractiveMarker int_marker;
         int_marker.header.frame_id = "map_calibrated";
         tf::Vector3 position((p2.x-p1.x)/2 + p1.x, (p2.y-p1.y)/2 + p1.y, 0);
         tf::pointTFToMsg(position, int_marker.pose.position);
         int_marker.scale = 1;
         int_marker.name = "button_" + boost::lexical_cast<std::string>(v1->id()) + "_" + boost::lexical_cast<std::string>(v2->id());
         int_marker.description = "Loop closure\n(Left Click to Toggle)";
         visualization_msgs::InteractiveMarkerControl control;
         control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
         control.name = int_marker.name + "_control";

         // Create the display options
         bool edge_status = getEdgeStatus((g2o::OptimizableGraph::Edge *)*edge_it);  // is the edge on or off?
         visualization_msgs::Marker edge_marker = MakeEdge(int_marker, p1, p2, edge_status);  // when are these updated????
         control.markers.push_back( edge_marker );
         visualization_msgs::Marker switch_marker = MakeSwitch( int_marker, p1, p2, edge_status );
         control.markers.push_back( switch_marker );
         control.always_visible = true;
         int_marker.controls.push_back(control);

         server_->insert(int_marker);
         server_->setCallback(int_marker.name, boost::bind(&G2OSolver::processFeedback, this,_1));

         // Save where appropriate ...
         loop_closure_status_map_.insert( std::make_pair<edge_pair_t,bool>(current_edge,edge_status) );
         loop_closure_markers_.insert( std::make_pair<edge_pair_t,visualization_msgs::InteractiveMarker>(current_edge,int_marker) );
         loop_closure_edges_.insert( std::make_pair<edge_pair_t, g2o::OptimizableGraph::Edge *>(current_edge, (g2o::OptimizableGraph::Edge *)*edge_it) );
       }
       else
       {
         // Probably where you update the marker positions ...
       }
       server_->applyChanges();
     }
     else { // normal edges
       loop_edge.points.clear();
       loop_edge.points.push_back(p1);
       loop_edge.points.push_back(p2);
       loop_edge.id = id++;
       bool status = getEdgeStatus((g2o::OptimizableGraph::Edge*)*edge_it);
       loop_edge.color.a = 1.0;
       if(!status)
        loop_edge.color.a = 0.25;
       marray.markers.push_back(visualization_msgs::Marker(loop_edge));
     }
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

/* Display the loop closure links */
/*void G2OSolver::getLoopClosures(visualization_msgs::MarkerArray &marray)
{
  edge_data_pt_t::iterator edge_it;
  int id=0;
 
  visualization_msgs::Marker loop_edge;
  loop_edge.header.frame_id = map_frame_id_;
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
 
  for(edge_it = active_edges_.begin(); edge_it != active_edges_.end(); ++edge_it)
  {
    g2o::VertexSE2* v1, *v2;
    v1 = dynamic_cast<g2o::VertexSE2 *>(edge_it->second->vertices()[0]);
    v2 = dynamic_cast<g2o::VertexSE2 *>(edge_it->second->vertices()[1]);
   
    geometry_msgs::Point p1, p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

    if(use_switchable_markers_)
    {
      edge_pair_t current_edge = std::make_pair<int,int>(v1->id(),v2->id());
      loop_status_t::iterator map_iter = loop_closure_status_map_.find(current_edge);
      if( map_iter == loop_closure_status_map_.end()  )
      {
        // New loop closure edge, need to create an interactive marker for it
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map_calibrated";
        tf::Vector3 position((p2.x-p1.x)/2 + p1.x, (p2.y-p1.y)/2 + p1.y, 0);
        tf::pointTFToMsg(position, int_marker.pose.position);
        int_marker.scale = 1;
        int_marker.name = "button_" + boost::lexical_cast<std::string>(v1->id()) + "_" + boost::lexical_cast<std::string>(v2->id());
        int_marker.description = "Loop closure\n(Left Click to Toggle)";
        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        control.name = int_marker.name + "_control";

        // Create the display options
        bool edge_status = getEdgeStatus(edge_it->second);
        visualization_msgs::Marker edge_marker = MakeEdge(int_marker, p1, p2, edge_status);
        control.markers.push_back( edge_marker );
        visualization_msgs::Marker switch_marker = MakeSwitch( int_marker, p1, p2, edge_status );
        control.markers.push_back( switch_marker );
        control.always_visible = true;
        int_marker.controls.push_back(control);

        server_->insert(int_marker);
        server_->setCallback(int_marker.name, boost::bind(&G2OSolver::processFeedback, this,_1));

        // Save where appropriate ...
        loop_closure_status_map_.insert( std::make_pair<edge_pair_t,bool>(current_edge,edge_status) );
        loop_closure_markers_.insert( std::make_pair<edge_pair_t,visualization_msgs::InteractiveMarker>(current_edge,int_marker) );
      }
    }
    else { // normal edges
      loop_edge.points.clear();
      loop_edge.points.push_back(p1);
      loop_edge.points.push_back(p2);
      loop_edge.id = id++;
      bool status = getEdgeStatus(edge_it->second);
      loop_edge.color.a = 1.0;
      if(!status)
        loop_edge.color.a = 0.25;
      marray.markers.push_back(visualization_msgs::Marker(loop_edge));
    }
 }
  server_->applyChanges();
}
*/

visualization_msgs::Marker G2OSolver::MakeEdge(visualization_msgs::InteractiveMarker &msg, geometry_msgs::Point &p1, geometry_msgs::Point &p2, bool status)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "map_calibrated";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 0.5;
  if(status)
    marker.color.a = 1.0;
  else
    marker.color.a = 0.25;
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  return marker;
}

visualization_msgs::Marker G2OSolver::MakeSwitch(visualization_msgs::InteractiveMarker &msg, geometry_msgs::Point &p1, geometry_msgs::Point &p2, bool status)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "map_calibrated";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.45;
  marker.scale.y = 0.45;
  marker.scale.z = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  if(status)
  {
    marker.color.g = 1.0;
  }
  else
  {
    marker.color.r = 1.0;
  }
  marker.color.a = 1.0;
  marker.pose.position.x = (p2.x-p1.x)/2+p1.x;
  marker.pose.position.y = (p2.y-p1.y)/2+p1.y;
  return marker;
}


void G2OSolver::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  bool bOK = true;

  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    ROS_ERROR("Got click from marker %s", feedback->marker_name.c_str());

    // Find the edge, toggle the status, optimize
    // button_ID1_ID2
    std::string::size_type n1, n2;
    std::string name = feedback->marker_name;
    n1 = name.find("_");
    std::string id = name.substr(n1+1);
    n2 = id.find("_");

    ROS_ERROR("IDs: %s and %s",id.substr(0,n2).c_str(),id.substr(n2+1,std::string::npos).c_str());

    std::string i1 = id.substr(0,n2);
    std::string i2 = id.substr(n2+1,std::string::npos);

    int id1 = atoi(i1.c_str());
    int id2 = atoi(i2.c_str());

    // toggle it
    edge_pair_t edge = std::make_pair<int,int>(id1,id2);
    bool current_state = loop_closure_status_map_.find(edge)->second;
    loop_closure_status_map_.find(edge)->second = !current_state;

    visualization_msgs::InteractiveMarker marker = loop_closure_markers_.find(edge)->second;

    loop_edge_map_t::iterator it = loop_closure_edges_.find(edge);
     
    if(it == loop_closure_edges_.end())
    {
      ROS_ERROR("Edge %d to %d not found in active_edges", id1,id2);
      return;
    }
    
    // Update position and check status
    if(current_state == true)
    {
      ROS_INFO("Switching %d to %d OFF", id1, id2);
      marker.controls[0].markers[1].color.r=1.0;
      marker.controls[0].markers[1].color.g=0.0;
      turnEdgeOff(it->second);
    }
    else
    {
      ROS_INFO("Switching %d to %d ON", id1, id2);
      marker.controls[0].markers[1].color.r=0.0;
      marker.controls[0].markers[1].color.g=1.0;
      turnEdgeOn(it->second);
    }

    server_->insert(marker); // should overwrite
    server_->applyChanges();
    if(bOK)
    {
      ROS_ERROR("OPTIMIZING after switching edge %d to %d",id1,id2);
      optimizer_->initializeOptimization(0); // OFF is level 1 and will be ignored by the optimizer
      optimizer_->optimize(optimization_iterations_,online_optimization_);
      ROS_ERROR("DONE");
    }
  }
}

bool G2OSolver::getEdgeStatus(g2o::OptimizableGraph::Edge* edge)
{
  return true;
}

bool G2OSolver::turnEdgeOn(g2o::OptimizableGraph::Edge* e)
{
 e->setLevel(0);  
}

bool G2OSolver::turnEdgeOff(g2o::OptimizableGraph::Edge* e)
{
  e->setLevel(1);
}



