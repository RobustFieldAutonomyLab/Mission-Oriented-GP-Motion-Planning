#include "OMPLHelper.h"
#include "../include/YAMLReader.h"


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