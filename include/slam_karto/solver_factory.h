#include <slam_karto/slam_karto_solver.h>
#include <slam_karto/spa_solver.h>
#include <slam_karto/g2o/g2o_solver.h>
#include <slam_karto/g2o/vertigo_maxmix_solver.h>
#include <slam_karto/g2o/vertigo_switchable_solver.h>

#include <ros/ros.h>
class SolverFactory
{
  public:
    static SlamKartoSolver *NewSolver(const std::string &description)
    {
      if(description == "MAXMIX")
      {
        ROS_INFO("Using VertigoMaxMixSolver");
        return new VertigoMaxMixSolver;
      }
      if(description == "SWITCHABLE")
      {
        ROS_INFO("Using VertigoSwitchableSolver");
        return new VertigoSwitchableSolver();
      }
      if(description == "G2O")
      {
        ROS_INFO("Using G2OSolver");
        return new G2OSolver;
      }
      else
      {
        ROS_INFO("Using SpaSolver");
        return new SpaSolver;
      }  
    }
};
