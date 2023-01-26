#include <limits>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "../include/SignedDistanceField.h"
#include "../include/Visualization.h"
#include "../gpmp2/mission/Seafloor.h"
#include "../include/OMPLHelper.h"

struct GAParameter:OMPLParam{
    int generations_;
    int popsize_;
    double c_;
    int path_length_;
};

class GeneticAlgorithm {
public:
    GeneticAlgorithm(const char * file_name, GAParameter params);

    std::tuple<bool, std::vector<gtsam::Pose3>> Plan(const gtsam::Pose3& start,
                                             const gtsam::Pose3& goal);

    std::vector<gtsam::Pose3> ReturnLastPath() const;

    void PrintChromosome(const std::vector<gtsam::Pose3>& path) const;

    void PrintPathOfChromosome(const std::vector<gtsam::Pose3>& path) const;

    std::vector<gtsam::Pose3> GenerateSimplePath() const;

    std::vector<gtsam::Pose3> GenerateRandomPath() const;

    int CalculateFitness(const std::vector<gtsam::Pose3>& path) const;

    std::vector<gtsam::Pose3> Crossover() const;

    std::vector<gtsam::Pose3> Mutate() const;

    bool CheckPath(const std::vector<gtsam::Pose3>& path) const;

    void CheckIfNodesInPathAreAcceptable(const std::vector<gtsam::Pose3>& path) const;

private:
    std::vector<gtsam::Pose3> motions_;

    gtsam::Pose3 start_, goal_;
    size_t path_length_ = 30;

    int f_val = std::numeric_limits<int>::max();

    int generations_ = 10000;
    int popsize_ = 30;
    double c_ = 1.05;

    std::vector<std::vector<gtsam::Pose3>> paths_;
    std::vector<std::vector<gtsam::Pose3>> truepaths_;
    bool found_ = false;

    gpmp2::SignedDistanceField*sdf_;
    double dist_sdf_;

    gpmp2::Seafloor*sf_;
    double dist_sf_;

    double cell_size_;
    gtsam::Point3 origin_;


};
