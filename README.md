# sensorbox

A package for processing sensor data.

## Core C++ Library

### Prerequisites

| **Dependency** | **Version** | **Description** |
|----------------|-------------|-----------------|
| Eigen3 | >= 3.3.7 (< g++-10) or >= 3.3.9 (>= g++-10) | Linear Algebra Package |
| [mathbox](https://github.com/willat343/mathbox) | 0.4.0 | Math Package |
| [cppbox](https://github.com/willat343/cppbox) | >= 0.0.1 | C++ Package |

### Installation

It is recommended that you configure with `ccmake` (`sudo apt install cmake-curses-gui`) to see the various options. Otherwise use `cmake` instead of `ccmake` and set flags manually.

```bash
cd sensorbox
mkdir build && cd build
ccmake ..
make -j
sudo make install
```

### Uninstallation

```bash
cd build
sudo make uninstall
```

### Usage

Import the package into your project and add the dependency to your target `<target>` with:
```cmake
find_package(sensorbox REQUIRED)
target_link_libraries(<target> <PUBLIC|INTERFACE|PRIVATE> ${sensorbox_LIBRARIES})
target_include_directories(<target> SYSTEM <PUBLIC|INTERFACE|PRIVATE> ${sensorbox_INCLUDE_DIRS})
```

For more information, see [sensorbox/README.md](sensorbox/README.md) and documentation.

### Documentation

Documentation must be turned on by setting the `-DBUILD_DOCUMENTATION=ON` cmake argument.

To view the HTML documentation, open the `build/docs/html/index.html` file.

To view the LaTeX documentation, build it with:
```bash
cd build/docs/latex
make
```
Then open the file `refman.pdf`.

### Tests

Tests must be turned on by setting the `-DBUILD_TESTS=ON` cmake argument.

```bash
cd build
cmake -DBUILD_TESTS=ON ..
make -j
```

They can then be run with `ctest`:
```bash
ctest --test-dir test
```

For more explicit output, the test executables can be run directly from the build directory.

## Catkin Support

A `package.xml` is supplied to facilitate an isolated installation within a catkin workspace (e.g. for ROS applications).

### Prerequisites

Prerequisites of core C++ library plus the following:

| **Dependency** | **Version** | **Description** |
|----------------|-------------|-----------------|
| catkin | - | catkin build system |

### Installation

Clone or symlink the repository to the workspace's `src` directory, for example:
```bash
ln -s /path/to/sensorbox /path/to/catkin_ws/src
```

```bash
cd /path/to/catkin_ws
catkin build sensorbox
```

### Uninstallation

```bash
cd /path/to/catkin_ws
catkin clean sensorbox
```

### Usage

To use the package in a downstream project, one should add to their `package.xml`:
```xml
<depend>sensorbox</depend>
```
One can then either use the workspace's isolated installation or use the system installation otherwise.
Importing the dependency is then exactly the same as it would be in a non-catkin package as described above (do NOT rely on the `catkin` variables like `catkin_LIBRARIES` and `catkin_INCLUDE_DIRS`).

### Documentation

Documentation must be turned on by setting the `-DBUILD_DOCUMENTATION=ON` cmake argument. This can be done in catkin with:
```bash
catkin config --cmake-args -DBUILD_DOCUMENTATION=ON
```

### Tests

Tests must be turned on by setting the `-DBUILD_TESTS=ON` cmake argument. This can be done in catkin with:
```bash
catkin config --cmake-args -DBUILD_TESTS=ON
```
