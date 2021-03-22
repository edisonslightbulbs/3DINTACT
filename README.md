## INTACT: Toolkit for fast segmentation of tabletop interaction context

Fast segmentation for tabletop interaction context.

![](https://github.com/edisonslightbulbs/INTACToolkit/blob/main/doc/figures/concept.png?raw=true)  | ![](https://github.com/edisonslightbulbs/INTACToolkit/blob/main/doc/figures/pipeline.png?raw=true)
:-------------------------:|:-------------------------:

### Overview:

|   Platform |   Hardware	|  Dependencies 	|
|---	|---	|---	|
|   :white_square_button: Linux	|   :white_square_button: Azure Kinect 	| :white_square_button: [ gflags](https://github.com/gflags/gflags)	|
|| |  :white_square_button: [ glog ](https://github.com/google/glog)  	|
|| |  :white_square_button: [ Eigen ](https://gitlab.com/libeigen/eigen.git) |
||| :white_square_button:  [ Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) |

### Project structure

    .
    ├── cmake
    ├── docs
    │   └── figures
    ├── external
    │   ├── glad
    │   │   ├── include
    │   │   └── src
    │   └── submodules
    │       └── Azure-Kinect-Sensor-SDK
    ├── include
    │   ├── hardware
    │   ├── middleware
    │   ├── software
    │   │   ├── activity
    │   │   ├── context
    │   │   └── surface
    │   └── utility
    ├── scripts
    │   └── setup-azure-kinect
    │       └── resources
    ├── src
    │   ├── hardware
    │   ├── middleware
    │   ├── software
    │   │   ├── activity
    │   │   ├── context
    │   │   ├── routine
    │   │   └── surface
    │   └── utility
    └── tests

### \[1/4] Getting started

***

```bash
# clone repository with submodules using the --recursive parameter
git clone --recurse-submodules https://github.com/edisonslightbulbs/INTACToolkit.git -j8

# if already cloned without the --recurse-submodules parameter
cd INTACToolkit
git submodule update --init --recursive
```

### \[2/4] Installing the dependencies

***

###### 1. The depth engine

A dated [Depth Engine](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) binary is provided and can be installed by running this [`install-depthengine.sh`](./scripts/setup-azure-kinect/) helper script. Please follow [ this hyperlink ](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) for the official step-by-step on how to get the most up-to-date depth engine.

###### 2. The USB rules

For convenience, the USB rules can be installed by running this [`install-usb-rules.sh`](./scripts/setup-azure-kinect/) helper script.

###### 3. Shared system libraries

To install the shared system dependencies, run this [`install-project-dependencies.sh`](./scripts/setup-azure-kinect/) helper script.

### \[3/4] Developer options in project [`CMakeLists.txt`](https://github.com/edisonslightbulbs/kinect-SAR/blob/master/CMakeLists.txt) file

***

###### option 1

```cpp
# recursively pull-update all project submodules (default behaviour is ON)
option(GIT_SUBMODULE "Check submodules during build" OFF)
```

###### option 2

```cpp
# build the Azure-Kinect-Sensor-SDK submodule (default behaviour is ON)
option(K4A_SDK_BUILD "Check submodules during build" OFF)
```

###### option 3

```cpp
# run target (default behaviour is OFF)
option(RUN "execute target" ON)
```

#### \[4/4] Building project and executing target

***

1.  from the project/root directory

```bash
mkdir build && cd build || return
cmake ..
make
```

2.  from the project root directory

```bash
# change to project binary directory
cd build/bin

# run target
./main

# to stdout logs [ optional ]
./main --logtostderr=1
```
