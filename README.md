# UUVPlanning

GP Motion Planning for 3D UUV
# Install
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 
```bash
sudo apt-get install cmake
```
- [Boost](http://www.boost.org/users/download/) >=1.65 < 1.74
```bash
sudo apt-get install libboost-all-dev
```

- [GTSAM](https://github.com/borglab/gtsam.git) 4.0.0
```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout wrap-export
mkdir build && cd build
cmake ..
make check  # optional, run unit tests
sudo make install
```

- [ YAML-cpp](https://github.com/jbeder/yaml-cpp.git) 
```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd 
mkdir build && cd build
cmake ..
make
sudo make install
```

- [edt](https://github.com/seung-lab/euclidean-distance-transform-3d)
```bash
git clone https://github.com/seung-lab/euclidean-distance-transform-3d.git
mkdir UUVPlanning/third_party/edt
cp euclidean-distance-transform-3d/cpp/edt.hpp UUVPlanning/third_party/edt
cp euclidean-distance-transform-3d/cpp/threadpool.h UUVPlanning/third_party/edt

In file threadpool.h
change line 73 to:
inline void ThreadPool::start(size_t threads) {

In file edt.hpp
change line 144 to:
inline void squared_edt_1d_parabolic(
change line 225 to:
inline void squared_edt_1d_parabolic(
change line 295 to:
inline void _squared_edt_1d_parabolic(

```

- [matlot++](https://github.com/alandefreitas/matplotplusplus)
```bash
cd UUVPlanning/third_party
git clone https://github.com/alandefreitas/matplotplusplus.git
```

# For Benchmark Testing
- [OMPL](https://ompl.kavrakilab.org/index.html)
- [STOMP](https://github.com/ros-industrial/stomp.git)
```bash
git clone https://github.com/ros-industrial/stomp.git
cp -r stomp/include/stomp UUVPlanning/benchmark/stomp/include
cp stomp/src/stomp.cpp UUVPlanning/benchmark/stomp/src
cp stomp/src/utils.cpp UUVPlanning/benchmark/stomp/src
In file stomp/src/stomp.cpp
change line 185 to:
Eigen::MatrixXd& parameters_optimized, double&cost)
change line 193 to:
  return solve(parameters_optimized_, parameters_optimized, cost);
change line 196 to:
bool Stomp::solve(const Eigen::VectorXd& first, const Eigen::VectorXd& last, Eigen::MatrixXd& parameters_optimized, double&cost)
change line 205 to:
  return solve(start, end, parameters_optimized, cost);
change line 208 to:
bool Stomp::solve(const Eigen::MatrixXd& initial_parameters, Eigen::MatrixXd& parameters_optimized, double&cost)
change line 283 to:
  cost = current_lowest_cost_;
In file stomp/include/stomp/stomp.h
change line 53 to:
  bool solve(const std::vector<double>& first, const std::vector<double>& last, Eigen::MatrixXd& parameters_optimized, double& cost);
change line 62 to:
  bool solve(const Eigen::VectorXd& first, const Eigen::VectorXd& last, Eigen::MatrixXd& parameters_optimized, double & cost);
change line 70 to:
  bool solve(const Eigen::MatrixXd& initial_parameters, Eigen::MatrixXd& parameters_optimized, double & cost);
```

# GPMP 
The GP Motion Planning code in GPMP2 doc is developed based on [GPMP2](https://github.com/borglab/gpmp2.git).
