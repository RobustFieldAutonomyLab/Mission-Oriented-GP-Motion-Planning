#include "OMPLHelper.h"
#include "../include/YAMLReader.h"
inline OMPLParameter readOMPLParamYAML(YAML::Node config){
    OMPLParameter param;
    auto planner = config["planner"];
    param.vehicle_size = planner["vehicle_size"].as<double>();

    YAML::Node dynamics = planner["dynamics"];
    param.w_vd = dynamics["sigma"].as<double>();

    YAML::Node obstacle = planner["obstacle"];
    param.dist_sdf = obstacle["epsilon_dist"].as<double>();
    param.w_sdf = obstacle["cost_sigma"].as<double>();

    YAML::Node seafloor = planner["seafloor"];
    param.seafloor_mission = seafloor["seafloor_mission"].as<bool>();
    if (!param.seafloor_mission){
        param.w_sf = 0;
        param.dist_sf = 0;
    }
    else{
        param.w_sf = seafloor["cost_sigma"].as<double>();
        param.dist_sf = seafloor["epsilon_dist"].as<double>();
    }

    auto map = config["map"];
    param.origin = map["origin"].as<Vector3>();
    param.cell_size = map["cell_size"].as<double>();
    param.cell_size_z = map["cell_size_z"].as<double>();
    param.sea_level = map["sea_level"].as<double>();

    YAML::Node ompl = config["ompl"];
    param.method = ompl["method"].as<string>();
    param.max_time = ompl["max_planning_time"].as<double>();
    param.cost_threshold = ompl["cost_threshold"].as<double>();
    return param;
}

int main(int argc/*argc*/, char *argv[] /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    YAML::Node config = YAML::LoadFile(argv[1]);
    OMPLParameter params = readOMPLParamYAML(config);

    auto path = config["path"];
    auto traj = config["trajectory"];
    auto visualize = config["visualization"];
    OMPLHelper env(path["seafloor_path"].as<string>(), params);

    auto s_rot = traj["start_rotation"].as<Vector3>();
    auto s_pos = traj["start_position"].as<Vector3>();

    auto e_rot = traj["end_rotation"].as<Vector3>();
    auto e_pos = traj["end_position"].as<Vector3>();

    Pose3 start_pose = Pose3(Rot3::RzRyRx(s_rot[2], s_rot[1], s_rot[0]),
                             Point3(s_pos[0], s_pos[1], s_pos[2]));

    Pose3 end_pose = Pose3(Rot3::RzRyRx(e_rot[2], e_rot[1], e_rot[0]),
                           Point3(e_pos[0], e_pos[1], e_pos[2]));

    if (env.plan(start_pose, end_pose) )
    {
        if (visualize["visualize"].as<bool>()){
            PLOT_TYPE tp;
            if (visualize["downsize_mesh"].as<bool>())
                tp = DOWNSIZE_MESH;
            else
                tp = MESH;
        env.recordSolution(tp, "result.txt");
        }
    }

    return 0;
}