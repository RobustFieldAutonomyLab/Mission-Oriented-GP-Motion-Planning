#pragma once

#include "../obstacle/SDFexception.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

namespace gpmp2{
class Seafloor {

public:
    typedef std::pair<size_t, size_t> index;
    typedef std::pair<double, double> float_index;

private:
    gtsam::Point3 origin_;
    size_t field_rows_, field_cols_;
    double cell_size_;
    gtsam::Matrix data_;

public:
    Seafloor() {}

    Seafloor(const gtsam::Point3& origin, double cell_size, const gtsam::Matrix& data) :
        origin_(origin), cell_size_(cell_size),
            field_cols_(data.cols()), field_rows_(data.rows()),
                data_(data) {}

    ~Seafloor() {}

    gtsam::Matrix getData(){
        return data_;
    }

    double getRowScale(){return field_rows_ * cell_size_;}

    double getColScale(){return field_cols_ * cell_size_;}

    inline double getDistance(const gtsam::Point3& point) const {
        const float_index pidx = convertPoint2toCell(gtsam::Point2(point.x(), point.y()));
        const double lr = floor(pidx.first), lc = floor(pidx.second);
        const double hr = lr + 1.0, hc = lc + 1.0;
        const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
                hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc);
        const double zlrhc = data_(lri, hci), zhrhc = data_(hri, hci),
                zlrlc = data_(lri, lci), zhrlc = data_(hri, lci);
        const double zph = zlrhc + (zhrhc - zlrhc) * (pidx.first - lr);
        const double zpl = zlrlc + (zhrlc - zlrlc) * (pidx.first - lr);
        const double zp = zpl + (zph - zpl) * (pidx.second - lc);
//        std::cout<< "lr, hr, lc, hc:" << lr<<", "<<hr<<
//        ", "<<lc<<", "<<hc<<std::endl;
//        std::cout<< "zp:" << zp <<std::endl;
        return (point.z() - zp);
    }

    inline float_index convertPoint2toCell(const gtsam::Point2& point) const {
        if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
            point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_) ) {
            throw SDFQueryOutOfRange();
        }

        const double col = (point.x() - origin_.x()) / cell_size_;
        const double row = (point.y() - origin_.y()) / cell_size_;
        return std::make_pair(row, col);
    }

};

}