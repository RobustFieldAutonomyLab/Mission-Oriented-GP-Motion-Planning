#include "STOMPHelper.h"
#include "../include/YAMLReader.h"
#include <matplot/matplot.h>
#include "stomp/stomp.h"

/**
 * @brief Creates a STOMP configuration object with default parameters.
 * @return A STOMP configuration object
 */

void recordSolution(Trajectory traj, Matrix seafloor_map, Point3 origin, double cell_size, PLOT_TYPE tp, std::string file_name)
{
    plotEvidenceMap3D(seafloor_map,origin.x(),origin.y(),cell_size,tp);
    int num_cols = traj.cols();
    std::vector<double> X, Y, Z;

    for (int i = 0; i < num_cols; i++)
    {
        X.push_back(traj(0, i));
        Y.push_back(traj(1, i));
        Z.push_back(traj(2, i));
    }
    matplot::hold(matplot::on);
    matplot::plot3(X, Y, Z,"-ob");
    matplot::show();
}

stomp::StompConfiguration create3DOFConfiguration(YAML::Node node)
{
    //! [Create Config]
    using namespace stomp;

    StompConfiguration c;
    auto traj = node["trajectory"];
    auto stomp = node["stomp"];
    c.num_timesteps = traj["total_time_step"].as<int>();
    c.num_iterations = stomp["num_iterations"].as<int>();
    c.num_dimensions = 3;
    c.delta_t = traj["delta_t"].as<double>();
    c.control_cost_weight = stomp["control_cost_weight"].as<double>();

    int method_id = stomp["initialization_method"].as<int>();
    switch(method_id){
        case 1:
            c.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
            break;
        case 2:
            c.initialization_method = TrajectoryInitializations::CUBIC_POLYNOMIAL_INTERPOLATION;
            break;
        case 3:
            c.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST;
            break;
        default:
            c.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
            break;
    }
    c.num_iterations_after_valid = stomp["num_iterations_after_valid"].as<int>();
    c.num_rollouts = stomp["num_rollouts"].as<int>();
    c.max_rollouts = stomp["max_rollouts"].as<int>();
    //! [Create Config]
    return c;
}

int main(int argc, char* argv[])
{
    YAML::Node config = YAML::LoadFile(argv[1]);
    STOMPParameter params = readSTOMPParamYAML(config);

    auto path = config["path"];
    auto traj = config["trajectory"];
    auto visualize = config["visualization"];
    stomp::TaskPtr task(new STOMPHelper("../" + path["seafloor_path"].as<string>(), params));

    stomp::StompConfiguration stomp_config = create3DOFConfiguration(config);
    stomp::Stomp stomp(stomp_config, task);
    //! [Create STOMP]

    //! [Solve]
    /**< Optimizing a trajectory close enough to the bias is produced **/
    Trajectory optimized;
    Point3 start_pt = traj["start_position"].as<Vector3>();
    Point3 end_pt = traj["end_position"].as<Vector3>();
    if (stomp.solve(start_pt, end_pt, optimized))
    {
        std::cout << "STOMP succeeded" << std::endl;
    }
    else
    {
        std::cout << "A valid solution was not found" << std::endl;
        return -1;
    }
    if(visualize["visualize"].as<bool>()){
        PLOT_TYPE tp;
        if (visualize["downsize_mesh"].as<bool>()){
            tp = PLOT_TYPE::DOWNSIZE_MESH;
        }
        else
            tp = PLOT_TYPE::MESH;
        Matrix data = loadSeaFloorData("../" + path["seafloor_path"].as<string>());
        recordSolution(optimized, data, params.origin, params.cell_size, tp, "result.txt");

    }

return 0;
}