# sensorbox

A package for processing sensor data.

## Core C++ Library

### Prerequisites

- [mathbox](https://github.com/willat343/mathbox)

### Installation

It is recommended that you configure with `ccmake` (`sudo apt install cmake-curses-gui`) to see the various options. Otherwise use `cmake` instead of `ccmake` and set flags manually.

```bash
cd sensorbox
mkdir build && cd build
ccmake ..
sudo make install
```

### Uninstallation

```bash
cd build
sudo make uninstall
```

## Catkin Support

A catkin wrapper is available to facilitate easy integration with the catkin build system (e.g. for ROS applications). To use sensorbox with catkin, simply clone or symlink the sensorbox repository to the catkin workspace `src` directory:
```bash
ln -s /path/to/sensorbox /path/to/catkin_ws/src
```

The [mathbox](https://github.com/willat343/mathbox) repository must also exist in the catkin workspace.

Your project can then depend on `sensorbox_catkin` in the `package.xml` and `sensorbox_catkin` can be added to `COMPONENTS`, e.g. `find_package(catkin REQUIRED COMPONENTS sensorbox_catkin)`.

## Changelog

* [10.07.2024] Migrated IMU classes and functionality from [serpent](https://github.com/jpl-eels/serpent/tree/develop)
