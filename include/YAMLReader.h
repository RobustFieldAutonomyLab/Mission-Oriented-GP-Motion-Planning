#include "yaml-cpp/yaml.h"
#include "Planning3DUUV.h"

namespace YAML {
    template<>
    struct convert<Vector3> {
        static Node encode(const Vector3& rhs) {
            Node node;
            node.push_back(rhs(0));
            node.push_back(rhs(1));
            node.push_back(rhs(2));
            return node;
        }

        static bool decode(const Node& node, Vector3& rhs) {
            if(!node.IsSequence() || node.size() != 3) {
                return false;
            }

            rhs = Vector3(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
            return true;
        }
    };
}

inline Planning3DUUVParameter readParamYAML(YAML::Node planner){
    Planning3DUUVParameter param;

    param.vehicle_size = planner["vehicle_size"].as<double>();
    param.check_inter  = planner["check_inter"].as<int>();
    param.max_iter     = planner["max_iter"].as<int>();
    if (planner["Qc"])
        param.Qc = planner["Qc"].as<vector<double>>();

    YAML::Node dynamics = planner["dynamics"];
    param.use_vehicle_dynamics = dynamics["use_vehicle_dynamics"].as<bool>();
    param.dynamics_sigma = dynamics["sigma"].as<double>();

    YAML::Node obstacle = planner["obstacle"];
    param.obstacle_epsilon_dist = obstacle["epsilon_dist"].as<double>();
    param.obstacle_cost_sigma = obstacle["cost_sigma"].as<double>();

    YAML::Node seafloor = planner["seafloor"];
    param.seafloor_mission = seafloor["seafloor_mission"].as<bool>();
    if(param.seafloor_mission){
        param.seafloor_cost_sigma = seafloor["cost_sigma"].as<double>();
        param.seafloor_dist = seafloor["epsilon_dist"].as<double>();

        auto roi_d = seafloor["region_of_interest"].as<vector<vector<double>>>();
        for (auto p : roi_d){
            param.region_of_interest.emplace_back(p[0], p[1]);
        }
    }
    YAML::Node current = planner["current"];
    param.use_current = current["use_current"].as<bool>();
    return param;
}