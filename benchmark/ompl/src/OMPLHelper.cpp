#include "OMPLHelper.h"
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

ob::OptimizationObjectivePtr multiObjective(const ob::SpaceInformationPtr& si,
                                            gpmp2::Seafloor sf, double dist_sf,
                                            gpmp2::SignedDistanceField sdf, double dist_sdf,
                                            double cost_thres = 100,
                                            double w_vd = 1, double w_sdf = 1, double w_sf = 1){
    ob::OptimizationObjectivePtr vehicleDynamicsObj(new vehicleDynamicsObjective(si));
    ob::OptimizationObjectivePtr seafloorFollowingObj(new seafloorFollowingObjective(si,sf, dist_sf));
    ob::OptimizationObjectivePtr signedDistanceFieldObj(new signedDistanceFieldObjective(si, sdf, dist_sdf));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(vehicleDynamicsObj, w_vd);
    opt->addObjective(seafloorFollowingObj, w_sf);
    opt->addObjective(signedDistanceFieldObj, w_sdf);
    opt->setCostThreshold(ob::Cost(cost_thres));

    return ob::OptimizationObjectivePtr(opt);
}

OMPLHelper::OMPLHelper(const std::string& file_name, OMPLParameter params):
    vehicle_size_(params.vehicle_size), origin_(params.origin),
        cell_size_(params.cell_size), method_(params.method),cell_size_z_(params.cell_size_z),
        max_time_(params.max_time), sea_level_(params.sea_level),
        dist_sdf_(params.dist_sdf), dist_sf_(params.dist_sf),
        use_objective_(params.use_objective){
    bool ok = false;
    double s_max, s_min;
    try
    {
        gtsam::Matrix data = loadSeaFloorData(file_name);
        sf_ = new gpmp2::Seafloor(params.origin, params.cell_size, data);
        sdf_ = buildSDF(cell_size_, params.cell_size_z, origin_, data, params.sea_level);
        s_max = data.maxCoeff();
        s_min = data.minCoeff();
        ok = true;
    }
    catch (ompl::Exception &ex)
    {
        OMPL_ERROR("Unable to load %s.\n%s", &file_name, ex.what());
    }
    if (ok)
    {
//        auto space(std::make_shared<ob::RealVectorStateSpace>());
        auto space(std::make_shared<ob::SE3StateSpace>());

        double maxX = params.origin.x() + sf_->getColScale();
        double maxY = params.origin.y() + sf_->getRowScale();
        double maxZ = params.origin.z() +
                int(fmax(s_max, params.sea_level)
                - fmin(s_min, params.origin.z())) ;

        corner_ = gtsam::Point3(maxX, maxY, maxZ);

        ob::RealVectorBounds bounds(3);
        bounds.setLow(0,params.origin.x());
        bounds.setHigh(0,maxX);

        bounds.setLow(1,params.origin.y());
        bounds.setHigh(1,maxY);

        bounds.setLow(2,params.origin.z());
        bounds.setHigh(2,maxZ);
        space->setBounds(bounds);

        ss_ = std::make_shared<og::SimpleSetup>(space);

        // set state validity checking for this space
        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
//        space->setup();
//        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);

        // set optimization objective
        if (use_objective_)
        ss_->setOptimizationObjective(multiObjective(ss_->getSpaceInformation(), *sf_, dist_sf_,
                             *sdf_, dist_sdf_, params.cost_threshold, params.w_vd, params.w_sdf, params.w_sf));

        if(method_ == "RRTStar")
            ss_->setPlanner(std::make_shared<og::RRTstar>(ss_->getSpaceInformation()));
    }
}

bool OMPLHelper::plan(gtsam::Pose3 start_pt, gtsam::Pose3 end_pt){
    if (!ss_)
        return false;

    ob::ScopedState<ob::SE3StateSpace>start(ss_->getSpaceInformation());
    start->setX(start_pt.translation().x());
    start->setY(start_pt.translation().y());
    start->setZ(start_pt.translation().z());
    auto rot_s = start_pt.rotation().quaternion();
    start->rotation().setAxisAngle(rot_s(0), rot_s(1),rot_s(2), rot_s(3));

    ob::ScopedState<ob::SE3StateSpace>goal(ss_->getSpaceInformation());
    goal->setX(end_pt.translation().x());
    goal->setY(end_pt.translation().y());
    goal->setZ(end_pt.translation().z());
    auto rot_g = end_pt.rotation().quaternion();
    goal->rotation().setAxisAngle(rot_g(0), rot_g(1),rot_g(2), rot_g(3));
    ss_->setStartAndGoalStates(start, goal);
    // generate a few solutions; all will be added to the goal;
    ss_->solve(max_time_);

    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    if (ss_->haveSolutionPath())
    {
//        ss_->simplifySolution();

        og::PathGeometric &p = ss_->getSolutionPath();
//        ss_->getPathSimplifier()->simplifyMax(p);
//        ss_->getPathSimplifier()->smoothBSpline(p);
//        ss_->getPathSimplifier()->shortcutPath(p);
        int goal_id = p.getStateCount() - 1;
        auto goal_state = p.getState(goal_id)->as<ob::SE3StateSpace::StateType>();
        double x_goal = goal_state->getX();
        double y_goal = goal_state->getY();
        if(abs(x_goal - end_pt.translation().x()) > 1 || abs(y_goal - end_pt.translation().y()) > 1)
            return false;

        return true;
    }

    return false;
}

void OMPLHelper::recordSolution(bool visualize, PLOT_TYPE tp, std::string file_name, double count)
{
    if (!ss_ || !ss_->haveSolutionPath())
        return;
    if (visualize){
        plotEvidenceMap3D(sf_->getData(),origin_.x(),origin_.y(),cell_size_,tp);

        matplot::hold(matplot::on);
    }

    og::PathGeometric &p = ss_->getSolutionPath();
//    p.interpolate(300);


    auto cost = p.cost(ss_->getOptimizationObjective());
    std::vector<double> opt_x, opt_y, opt_z;
    std::ofstream file;
    file.open (file_name);
    file << "Computational Time: "<<count<<std::endl;
    file << "Traj Length: "<<p.length()<<std::endl;
    file << "Total Timestamp: "<<p.getStateCount()<<std::endl;
    file << "Total Cost: "<<cost.value()<<std::endl;
    file << "x, y, z"<<std::endl;
    for (std::size_t i = 0; i < p.getStateCount(); ++i)
    {
        opt_x.push_back(p.getState(i)->as<ob::SE3StateSpace::StateType>()->getX());
        opt_y.push_back(p.getState(i)->as<ob::SE3StateSpace::StateType>()->getY());
        opt_z.push_back(p.getState(i)->as<ob::SE3StateSpace::StateType>()->getZ());
        file << opt_x[i] << ", "<< opt_y[i] << ", " <<opt_z[i] <<std::endl;
    }
    file.close();
    if(visualize){
        auto l = matplot::plot3(opt_x, opt_y, opt_z,"-ob");
        matplot::show();
    }

}

bool OMPLHelper::isStateValid(const ompl::base::State *state) const
{
    double x = state->as<ob::SE3StateSpace::StateType>()->getX();
    double y = state->as<ob::SE3StateSpace::StateType>()->getY();
    double z = state->as<ob::SE3StateSpace::StateType>()->getZ();
    if(x < origin_.x() || y < origin_.y() || z < origin_.z())
        return false;
    if(x+cell_size_ > corner_.x() || y+cell_size_ > corner_.y() || z+cell_size_z_ > corner_.z() || z > sea_level_)
        return false;
//    else
//        return true;
    double dist = sf_->getDistance(gtsam::Point3(x, y, z));
    if (dist > vehicle_size_)
        return true;
    else
        return false;
}