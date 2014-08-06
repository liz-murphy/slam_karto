/* Extends ScanSolver and forces everything to have a graph publisher */
#ifndef SLAM_KARTO_SOLVER_H
#define SLAM_KARTO_SOLVER_H

#include <open_karto/Mapper.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

class SlamKartoSolver : public karto::ScanSolver
{
  public:
    virtual void publishGraphVisualization(visualization_msgs::MarkerArray &marray)=0;
    bool setMapFrame(std::string &map_frame){map_frame_id_=map_frame;return true;};
  protected:
    std::string map_frame_id_;
};

#endif
