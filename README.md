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
mkdir build && cd build
cmake ..
make
sudo make install
```

# GPMP 
The GP Motion Planning code in GPMP2 doc is developed based on [GPMP2](https://github.com/borglab/gpmp2.git).