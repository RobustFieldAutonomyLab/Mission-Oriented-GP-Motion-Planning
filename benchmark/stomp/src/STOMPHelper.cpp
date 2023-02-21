#include "../include/STOMPHelper.h"

STOMPHelper::STOMPHelper(const std::string& file_name,
                    const STOMPParameter& params)
        : vehicle_size_(params.vehicle_size), origin_(params.origin),
        cell_size_(params.cell_size), dist_sdf_(params.dist_sdf),
        w_sdf_(params.w_sdf), dist_sf_(params.dist_sf), w_sf_(params.w_sf)
{
    stomp::generateSmoothingMatrix(params.total_time_steps, params.delta_t, smoothing_M_);
    srand(time(0));

    gtsam::Matrix data = loadSeaFloorData(file_name);
    sf_ = new gpmp2::Seafloor(origin_, cell_size_, data);
    sdf_ = buildSDF(cell_size_, params.cell_size_z, origin_, data, params.sea_level);

    double maxX = origin_.x() + sf_->getColScale() * cell_size_;
    double maxY = origin_.y() + sf_->getRowScale() * cell_size_;
    double maxZ = origin_.z() +
                  int(params.sea_level - fmin(data.minCoeff(), origin_.z()));

    corner_ = gtsam::Point3(maxX, maxY, maxZ);

    std::vector sd{params.std_dev(0), params.std_dev(1), params.std_dev(2)};
    std_dev_ = sd;
}

bool STOMPHelper::generateNoisyParameters(const Eigen::MatrixXd& parameters,
                             std::size_t start_timestep,
                             std::size_t num_timesteps,
                             int iteration_number,
                             int rollout_number,
                             Eigen::MatrixXd& parameters_noise,
                             Eigen::MatrixXd& noise)
{
    double rand_noise;
    for (std::size_t d = 0; d < parameters.rows(); d++)
    {
        for (std::size_t t = 0; t < parameters.cols(); t++)
        {
            rand_noise = static_cast<double>(rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1);  // 0 to 1
            rand_noise = 2 * (0.5 - rand_noise);
            noise(d, t) = rand_noise * std_dev_[d];
        }
    }

    parameters_noise = parameters + noise;

    return true;
}

bool STOMPHelper::computeCosts(const Eigen::MatrixXd& parameters,
                  std::size_t start_timestep,
                  std::size_t num_timesteps,
                  int iteration_number,
                  Eigen::VectorXd& costs,
                  bool& validity)
{
    return computeNoisyCosts(parameters, start_timestep, num_timesteps, iteration_number, -1, costs, validity);
}

bool STOMPHelper::computeNoisyCosts(const Eigen::MatrixXd& parameters,
                       std::size_t start_timestep,
                       std::size_t num_timesteps,
                       int iteration_number,
                       int rollout_number,
                       Eigen::VectorXd& costs,
                       bool& validity)
{
    costs.setZero(num_timesteps);
    validity = true;

    for (std::size_t t = 0u; t < num_timesteps; t++)
    {
        gtsam::Point3 pt(parameters(0, t), parameters(1, t), parameters(2, t));
        double err = 0, err_sdf = 0, err_sf = 0;
        if(pt.x() < origin_.x() || pt.y() < origin_.y() || pt.z() < origin_.z()){
            validity = false;
            err = 100;
        }
        else if(pt.x()+cell_size_ > corner_.x() ||
                pt.y()+cell_size_ > corner_.y() ||
                        pt.z()+cell_size_ > corner_.z()){
            validity = false;
            err = 100;
        }
        else{
            double dist_sdf = sdf_->getSignedDistance(pt);
            double dist = sf_->getDistance(pt);

            if(dist_sdf < vehicle_size_ + dist_sdf_){
                err_sdf = vehicle_size_ + dist_sdf_ - dist;
            }
            if(dist < vehicle_size_){
                validity = false;
                err = dist;
            }
            else if(dist > dist_sf_){
                err_sf = dist;
            }

        }
        err = abs(err) + w_sdf_ * abs(err_sdf) + w_sf_ * abs(err_sf);
        costs(t) = err;
    }
    return true;
}

bool STOMPHelper::filterParameterUpdates(std::size_t start_timestep,
                            std::size_t num_timesteps,
                            int iteration_number,
                            const Eigen::MatrixXd& parameters,
                            Eigen::MatrixXd& updates)
{
    return smoothParameterUpdates(start_timestep, num_timesteps, iteration_number, updates);
//    return true;
}

bool STOMPHelper::smoothParameterUpdates(std::size_t start_timestep,
                            std::size_t num_timesteps,
                            int iteration_number,
                            Eigen::MatrixXd& updates)
{
    for (auto d = 0u; d < updates.rows(); d++)
    {
        updates.row(d).transpose() = smoothing_M_ * (updates.row(d).transpose());
    }

    return true;
}

