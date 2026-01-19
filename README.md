# v3.8.6
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Cloning the Repository
```bash
git clone https://github.com/AustinRover/lb_unitree.git
```

### Connecting to the Go1
Before running any examples, you must connect to the Unitree Go1 robot. See the official Trossen Robotics documentation for detailed setup instructions:

[Trossen Robotics - Unitree Go1 Getting Started](https://docs.trossenrobotics.com/unitree_go1_docs/getting_started.html)

### Notice
support robot: Go1

not support robot: Laikago, B1, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Dependencies
* [Unitree](https://www.unitree.com/download)
```bash
Legged_sport    >= v1.36.0
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [g++](https://gcc.gnu.org/) (version 8.3.0 or higher)


### Build
```bash
mkdir build
cd build
cmake ..
make
```

If you want to build the python wrapper, then replace the cmake line with:
```bash
cmake -DPYTHON_BUILD=TRUE ..
```

If can not find pybind11 headers, then add
```bash
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/pybind11/include)
```
at line 14 in python_wrapper/CMakeLists.txt.

If can not find msgpack.hpp, then
```bash
sudo apt install libmsgpack*
```

### Run

#### Cpp
Run examples with 'sudo' for memory locking.

#### Monitor GUI
The `monitor_gui` application provides a real-time graphical interface for monitoring the Go1's BMS data, joint torques, and running automated tests.

**Additional Dependencies:**
* [GLFW](https://www.glfw.org/) (for windowing)
* [OpenGL](https://www.opengl.org/)

Install on Ubuntu:
```bash
sudo apt install libglfw3-dev
```

**Running the Monitor GUI:**
```bash
cd build
sudo ./monitor_gui
```

The GUI allows you to:
- Monitor battery state of charge, voltage, current, and power in real-time
- View individual cell voltages
- Monitor estimated joint torques for all 12 joints
- Run automated test sequences (Neutral, Floor, Squat, Walk & Turn)
- Log data to CSV files for analysis

#### Python
##### arm
change `sys.path.append('../lib/python/amd64')` to `sys.path.append('../lib/python/arm64')`

### AI Usage Statement
AI was used to generate all code in `src/monitor_gui.cpp` and `src/monitor.cpp`.
