#pragma once

#include "../obstacle/SDFexception.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>

namespace gpmp2{
class WaterCurrentGrid{
public:
    typedef boost::tuple<size_t, size_t, size_t> index;
    typedef boost::tuple<double, double, double> float_index;
    typedef boost::shared_ptr<WaterCurrentGrid> shared_ptr;

private:
    gtsam::Point3 origin_;
    size_t field_rows_, field_cols_, field_z_;
    double cell_size_;
    double cell_size_z_;

    //now only constrains horizontal current velocity, more detailed data to be added
    std::vector<gtsam::Matrix> data_u_;
    std::vector<gtsam::Matrix> data_v_;

public:
    WaterCurrentGrid() {}

    WaterCurrentGrid(const gtsam::Point3& origin, double cell_size, double cell_size_z, size_t field_rows,
                        size_t field_cols, size_t field_z) :
            origin_(origin), field_rows_(field_rows), field_cols_(field_cols),
            field_z_(field_z), cell_size_(cell_size), cell_size_z_(cell_size_z),
            data_u_(std::vector<gtsam::Matrix>(field_z)),
            data_v_(std::vector<gtsam::Matrix>(field_z)) {}

    WaterCurrentGrid(const gtsam::Point3& origin, double cell_size, double cell_size_z, size_t field_rows,
                     size_t field_cols, size_t field_z,
                     const std::vector<gtsam::Matrix> &u_grid, const std::vector<gtsam::Matrix> &v_grid) :
            origin_(origin), field_rows_(field_rows), field_cols_(field_cols),
            field_z_(field_z), cell_size_(cell_size), cell_size_z_(cell_size_z),
            data_u_(u_grid), data_v_(v_grid) {}

    ~WaterCurrentGrid() {}

    void initFieldDataU(size_t z_idx, const gtsam::Matrix& field_layer) {
        if (z_idx >= field_z_)
            throw std::runtime_error("[SignedDistanceField] matrix layer out of index");
        data_u_[z_idx] = field_layer;
    }

    void initFieldDataV(size_t z_idx, const gtsam::Matrix& field_layer) {
        if (z_idx >= field_z_)
            throw std::runtime_error("[SignedDistanceField] matrix layer out of index");
        data_v_[z_idx] = field_layer;
    }

    inline gtsam::Vector6 getVelocity(const gtsam::Point3 point) const {
        const float_index pidx = convertPoint3toCell(point);
        double current_u = get_current(pidx, data_u_);
        double current_v = get_current(pidx, data_v_);
        return (gtsam::Vector(6) << 0, 0, 0, current_u, current_v, 0).finished();
    }

    /// convert between point and cell corrdinate
    inline float_index convertPoint3toCell(const gtsam::Point3& point) const {
        // check point range
        if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
            point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_) ||
            point.z() < origin_.z() || point.z() > (origin_.z() + (field_z_-1.0)*cell_size_z_)) {

            throw SDFQueryOutOfRange();
        }

        const double col = (point.x() - origin_.x()) / cell_size_;
        const double row = (point.y() - origin_.y()) / cell_size_;
        const double z   = (point.z() - origin_.z()) / cell_size_z_;
        return boost::make_tuple(row, col, z);
    }

    /// Same as SignedDistanceField
    /// tri-linear interpolation
    inline double get_current(const float_index& idx, const std::vector<gtsam::Matrix>& data) const {
        const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>()), lz = floor(idx.get<2>());
        const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
        const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc), lzi = static_cast<size_t>(lz),
                hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
//        std::cout<< "lr lc lz hr hc hz: "<<lr<<" "<<lc<<" "<< lz<<" "<< hr<<" "<< hc<<" "<< hz<<std::endl;
        return
                (hr-idx.get<0>())*(hc-idx.get<1>())*(hz-idx.get<2>())*get_current(lri, lci, lzi, data) +
                (idx.get<0>()-lr)*(hc-idx.get<1>())*(hz-idx.get<2>())*get_current(hri, lci, lzi, data) +
                (hr-idx.get<0>())*(idx.get<1>()-lc)*(hz-idx.get<2>())*get_current(lri, hci, lzi, data) +
                (idx.get<0>()-lr)*(idx.get<1>()-lc)*(hz-idx.get<2>())*get_current(hri, hci, lzi, data) +
                (hr-idx.get<0>())*(hc-idx.get<1>())*(idx.get<2>()-lz)*get_current(lri, lci, hzi, data) +
                (idx.get<0>()-lr)*(hc-idx.get<1>())*(idx.get<2>()-lz)*get_current(hri, lci, hzi, data) +
                (hr-idx.get<0>())*(idx.get<1>()-lc)*(idx.get<2>()-lz)*get_current(lri, hci, hzi, data) +
                (idx.get<0>()-lr)*(idx.get<1>()-lc)*(idx.get<2>()-lz)*get_current(hri, hci, hzi, data);
    }

    inline double get_current(size_t r, size_t c, size_t z, const std::vector<gtsam::Matrix>& data) const {
        return data[z](r, c);
    }


    inline gtsam::Vector6 getVehicleVelocityCurrent(const gtsam::Point3 position, const gtsam::Vector6 velocity) const {
        gtsam::Vector6 v_current;
        try {
            v_current = getVelocity(position);
        } catch (SDFQueryOutOfRange&) {
//            cout<<"SDFQueryOutOfRange"<<position<<endl;
            v_current = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
        }
        return velocity + v_current;
    }

};



}

