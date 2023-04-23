#include "Planning3DUUV.h"
#include "YAMLReader.h"


using namespace std;

pair<vector<Pose3>, vector<Vector>> generateInitialTrajectory(const Pose3& start_pose, const Vector& start_vel,
                                                              const Pose3& end_pose, const Vector& end_vel,
                                                              const Vector& avg_vel, int total_time_step) {
    vector <Pose3> ps;
    vector <Vector> vs;
    for (int i = 0; i<=total_time_step; i++){
        Pose3 pose = Pose3(Rot3(),
                           Point3(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step,
                                  start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step,
                                  start_pose.z() * (total_time_step-i)/total_time_step + end_pose.z() * i/total_time_step));
        ps.push_back(pose);
        if (i == 0)
            vs.push_back(start_vel);
        else if(i == total_time_step)
            vs.push_back(end_vel);
        else
            vs.push_back(avg_vel);
    }
    return make_pair(ps, vs);
}

void run(string yaml_path){
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto traj = config["trajectory"];
    auto planner = config["planner"];
    auto map = config["map"];
    auto path = config["path"];
    auto visualize = config["visualization"];

    Planning3DUUV p3d(readParamYAML(planner));

    auto delta_t  = traj["delta_t"].as<double>();
    auto total_time_step = traj["total_time_step"].as<int>();
    auto s_rot = traj["start_rotation"].as<Vector3>();
    auto s_pos = traj["start_position"].as<Vector3>();
    auto s_vel = traj["start_velocity"].as<Vector3>();
    auto s_ang = traj["start_angular_velocity"].as<Vector3>();

    auto e_rot = traj["end_rotation"].as<Vector3>();
    auto e_pos = traj["end_position"].as<Vector3>();
    auto e_vel = traj["end_velocity"].as<Vector3>();
    auto e_ang = traj["end_angular_velocity"].as<Vector3>();
    auto a_vel = traj["average_velocity"].as<Vector3>();
    auto a_ang = traj["average_angular_velocity"].as<Vector3>();

    Pose3 start_pose = Pose3(Rot3::RzRyRx(s_rot[2], s_rot[1], s_rot[0]),
                             Point3(s_pos[0], s_pos[1], s_pos[2]));
    Vector start_vel, end_vel, avg_vel;
    Vector6 start_vel6, end_vel6, avg_vel6;
    start_vel6 << s_ang, s_vel;
    start_vel = start_vel6;

    Pose3 end_pose = Pose3(Rot3::RzRyRx(e_rot[2], e_rot[1], e_rot[0]),
                           Point3(e_pos[0], e_pos[1], e_pos[2]));
    end_vel6 << e_ang, e_vel;
    end_vel = end_vel6;

    avg_vel6 << a_ang, a_vel;
    avg_vel = avg_vel6;

    auto initial_config = generateInitialTrajectory(start_pose, start_vel,
                                                    end_pose, end_vel, avg_vel, total_time_step);
    Matrix seafloor_map = loadSeaFloorData(path["seafloor_path"].as<string>());
    if(!path["load_sdf"].as<bool>() && path["sdf_path"])
        seafloor_map = p3d.buildMap(map["cell_size"].as<double>(),
            map["cell_size_z"].as<double>(),
                    Point3(map["origin"].as<Vector3>()),
                    seafloor_map,
                    map["sea_level"].as<double>(),
                    map["use_boundary"].as<bool>());
    else
        seafloor_map = p3d.buildMap(map["cell_size"].as<double>(),
                                    map["cell_size_z"].as<double>(),
                                    Point3(map["origin"].as<Vector3>()),
                                    seafloor_map,
                                    map["sea_level"].as<double>(),
                                    path["sdf_path"].as<string>());


    auto current = planner["current"];
    if(current["use_current"].as<bool>()){
        double c_cell_size = current["cell_size"].as<double>();
        double c_cell_size_z = current["cell_size_z"].as<double>();
        Point3 c_origin = Point3(current["origin"].as<Vector3>());
        vector<Matrix> current_grid_u = loadCurrentData(path["current_u_path"].as<string>(),
                                    c_cell_size, c_cell_size_z,
                                    c_origin.z(), map["sea_level"].as<double>());
        vector<Matrix> current_grid_v = loadCurrentData(path["current_v_path"].as<string>(),
                                    c_cell_size, c_cell_size_z,
                                    c_origin.z(), map["sea_level"].as<double>());
        p3d.buildCurrentGrid(c_cell_size,c_cell_size_z,c_origin,
                             current_grid_u,current_grid_v);
    }

    auto result = p3d.optimize(initial_config.first,
                               initial_config.second,
                               delta_t,
                               path["out_path"].as<string>());
    vector<double> X, Y, Z;
    for (auto pose : result){
        X.push_back(pose.x());
        Y.push_back(pose.y());
        Z.push_back(pose.z());
    }
    if (visualize["visualize"].as<bool>()){
        Vector3 ori = map["origin"].as<Vector3>();
        PLOT_TYPE tp;
        if(visualize["downsize_mesh"].as<bool>())
            tp = DOWNSIZE_MESH;
        else
            tp = MESH;
        plotEvidenceMap3D(seafloor_map,ori(0),
                          ori(1),map["cell_size"].as<double>(),tp);
        matplot::hold(matplot::on);
        auto l = matplot::plot3(X, Y, Z,"-ob");
        matplot::show();
    }
}

int main(int argc, char *argv[]){
    run(argv[1] );
}