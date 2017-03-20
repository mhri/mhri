# mhri

## Overview

This is a meta package of Social HRI software framework (sHRI).

The sHRI package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author(s): Byeong-Kyu Ahn, Minsu Jang   
Maintainer: Byeong-Kyu Ahn, byeongkyu@gmail.com  
Affiliation: Robot Group, Korea Insitute of Industrial Technology**


## Installation

### Installation from Packages

Under the work

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using:

    $ cd catkin_ws/src
    $ mkdir mhri_workspace
    $ wstool init mhri_workspace https://github.com/mhri/mhri/blob/master/mhri/doc/mhri.rosinstall.kinetic.plus
    $ cd mhri_workspace
    $ wstool update -j8
    $ catkin_make or catkin build


## Documentation

You will find the documents for usage this software framework under **mhri_workspace/mhri_docs**.
