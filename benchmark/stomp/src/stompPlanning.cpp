#include "STOMPHelper.h"
#include "../include/YAMLReader.h"
#include <matplot/matplot.h>
#include "stomp/stomp.h"
#include <chrono>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
/**
 * @brief Creates a STOMP configuration object with default parameters.
 * @return A STOMP configuration object
 */

inline STOMPParameter readSTOMPParamYAML(YAML::Node config){
    STOMPParameter param;
    auto planner = config["planner"];
    param.vehicle_size = planner["vehicle_size"].as<double>();

    auto obstacle = planner["obstacle"];
    param.dist_sdf = obstacle["epsilon_dist"].as<double>();
    param.w_sdf = 1 / obstacle["cost_sigma"].as<double>();

    auto seafloor = planner["seafloor"];
    if(seafloor["seafloor_mission"].as<bool>()){
        param.dist_sf = seafloor["epsilon_dist"].as<double>();
        param.w_sf = 1 / seafloor["cost_sigma"].as<double>();
    }
    else{
        param.dist_sf = 0;
        param.w_sf = 0;
    }

    auto map = config["map"];
    param.origin = map["origin"].as<Vector3>();
    param.cell_size = map["cell_size"].as<double>();
    param.cell_size_z = map["cell_size_z"].as<double>();
    param.sea_level = map["sea_level"].as<double>();

    auto traj = config["trajectory"];
    param.delta_t = traj["delta_t"].as<double>();
    param.total_time_steps = traj["total_time_step"].as<int>();

    auto stomp= config["stomp"];
    param.std_dev = stomp["std_dev"].as<Vector3>();

    return param;
}

void recordSolution(Trajectory traj, Matrix seafloor_map, double cost, Point3 origin, double cell_size,
                    double time, PLOT_TYPE tp, std::string file_name)
{
    int num_cols = traj.cols();
    double d = 0;
    for (int i = 1; i < num_cols; i++){
        Point3 p1(traj(0,i-1), traj(1,i-1), traj(2,i-1));
        Point3 p2(traj(0,i), traj(1,i), traj(2,i));
        d = d + gtsam::distance3(p1, p2);
    }
    std::vector<double> X, Y, Z;

    std::ofstream file;
    file.open (file_name);
    file << "Computational Time: " << time << std::endl;
    file << "Traj Length: " << d <<std::endl;
    file << "Total Timestamp: " << num_cols <<std::endl;
    file << "Total Cost: "<< cost * num_cols <<std::endl;
    file << "Average Cost: "<<cost << std::endl;
    file << "x, y, z"<<std::endl;

    for (int i = 0; i < num_cols; i++)
    {
        X.push_back(traj(0, i));
        Y.push_back(traj(1, i));
        Z.push_back(traj(2, i));
        file << traj(0, i) << ", "<< traj(1, i) << ", " <<traj(2,i) <<std::endl;
    }
    file.close();

    if(tp != OFF){
        plotEvidenceMap3D(seafloor_map,origin.x(),origin.y(),cell_size,tp);
        matplot::hold(matplot::on);
        matplot::plot3(X, Y, Z,"-ob");
        matplot::show();

    }

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
    c.exponentiated_cost_sensitivity = 1;
    c.num_iterations_after_valid = stomp["num_iterations_after_valid"].as<int>();
    c.num_rollouts = stomp["num_rollouts"].as<int>();
    c.max_rollouts = stomp["max_rollouts"].as<int>();
    //! [Create Config]
    return c;
}

void run(YAML::Node config, STOMPParameter params, string out_path){
    auto path = config["path"];
    auto traj = config["trajectory"];
    auto visualize = config["visualization"];
    stomp::TaskPtr task(new STOMPHelper(path["seafloor_path"].as<string>(), params));

    stomp::StompConfiguration stomp_config = create3DOFConfiguration(config);
    stomp::Stomp stomp(stomp_config, task);
    //! [Create STOMP]

    //! [Solve]
    /**< Optimizing a trajectory close enough to the bias is produced **/
    Trajectory optimized;
    Point3 start_pt = traj["start_position"].as<Vector3>();
    Point3 end_pt = traj["end_position"].as<Vector3>();
    double cost;
    auto start_time = std::chrono::high_resolution_clock::now();
    if (stomp.solve(start_pt, end_pt, optimized, cost))
    {
        std::cout << "STOMP succeeded" << std::endl;
        cost = cost / float(params.total_time_steps);
    }
    else
    {
        std::cout << "A valid solution was not found" << std::endl;
        optimized = zeros(params.total_time_steps,3);
        cost = cost * 100;
//        return -1;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    PLOT_TYPE tp;
    if(visualize["visualize"].as<bool>()){
        if (visualize["downsize_mesh"].as<bool>()){
            tp = PLOT_TYPE::DOWNSIZE_MESH;
        }
        else
            tp = PLOT_TYPE::MESH;
    }
    else
        tp = PLOT_TYPE::OFF;
//    Matrix data = loadSeaFloorData("../" + path["seafloor_path"].as<string>());
    Matrix data = loadSeaFloorData(path["seafloor_path"].as<string>());
    recordSolution(optimized, data, cost, params.origin, params.cell_size, elapsed_time.count(), tp, out_path);

}

int main(int argc, char* argv[])
{
    YAML::Node config = YAML::LoadFile(argv[1]);
    STOMPParameter params = readSTOMPParamYAML(config);

    auto stomp_param = config["stomp"];
    int  repeat_times = stomp_param["repeat_times"].as<int>();
    string out_dir = stomp_param["out_dir"].as<string>();
    int create_dir = mkdir(out_dir.data(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    for (int i = 0; i < repeat_times; i++){
        std::stringstream ss;
        ss<< out_dir <<"/"<<i<<".txt";
        run(config, params, ss.str());
    }

    return 0;
}