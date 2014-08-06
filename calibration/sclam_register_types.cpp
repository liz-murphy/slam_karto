#include <slam_karto/g2o/types_calibration.h>

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  using namespace std;

  bool init_types_calibration()
  {
    cerr << __PRETTY_FUNCTION__ << " called" << endl;
    Factory* factory = Factory::instance();
    factory->registerType("ROBOT_LASER_SCLAM", new HyperGraphElementCreator<g2o::RobotLaserSCLAM>);
    return true;
  }

} // end namespace

