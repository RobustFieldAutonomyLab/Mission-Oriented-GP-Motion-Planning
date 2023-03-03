#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

template < class MODEL, class SDF >
class Planning{
protected:
    size_t _dof;
    int _check_inter;

    double _cost_sigma;
    double _epsilon_dist;

    SDF * sdf;
    MODEL *robot;

    gtsam::noiseModel::Gaussian::shared_ptr Qc_model;

    gtsam::noiseModel::Isotropic::shared_ptr pose_fix;

    gtsam::noiseModel::Isotropic::shared_ptr vel_fix;

public:
    Planning(size_t dof, int check_inter, double Qc_val,
             double cost_sigma, double epsilon_dist) :
        _dof(dof), _check_inter(check_inter),
        _cost_sigma(cost_sigma), _epsilon_dist(epsilon_dist){
        gtsam::Matrix Qc = Qc_val * gtsam::Matrix::Identity(dof, dof);
        Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);

        pose_fix = gtsam::noiseModel::Isotropic::Sigma(dof, 0.0001);

        vel_fix = gtsam::noiseModel::Isotropic::Sigma(dof, 0.0001);

    }
    virtual ~Planning(){}

};