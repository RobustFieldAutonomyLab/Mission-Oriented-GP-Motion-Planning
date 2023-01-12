#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>


#include "../gpmp2/mission/Seafloor.h"
#include "gtsam/geometry/Pose3.h"

enum OMPLMethod{RRTStar, LBKPiece};

struct OMPLParam{
    OMPLMethod method_;

    gtsam::Point3 origin_;
    double cell_size_;
    double cell_size_z_;

    double vehicle_size_;

};

class OMPLHelper
{
public:
    OMPLHelper(const char *ppm_file, OMPLParam params);

    bool plan(gtsam::Pose3 start_pt, gtsam::Pose3 end_pt);

    void recordSolution();

private:
    bool isStateValid(const ompl::base::State *state) const;

    ompl::geometric::SimpleSetupPtr ss_;

    gtsam::Point3 origin_, corner_;

    double cell_size_;
    double vehicle_size_;

    gpmp2::Seafloor* sf_;

    OMPLMethod method_;

};