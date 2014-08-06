#include <slam_karto/g2o/calibration/types_sclam2d.h>

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  G2O_REGISTER_TYPE_GROUP(calibration);
  G2O_REGISTER_TYPE(ROBOT_LASER_SCLAM, RobotLaserSCLAM);

} // end namespace
