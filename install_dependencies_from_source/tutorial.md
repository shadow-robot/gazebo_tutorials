# Install dependencies from source on Ubuntu

This tutorial will go through the process of installing some of Gazebo's
dependencies from source. The dependencies listed here are all maintained by
the Gazebo team and often new features in Gazebo are tied to new features in
these libraries.

These libraries are:

* [SDFormat](http://sdformat.org/)
* [ignition-cmake](http://ignitionrobotics.org/libraries/cmake)
* [ignition-common](http://ignitionrobotics.org/libraries/common)
* [ignition-fuel-tools](http://ignitionrobotics.org/libraries/fuel-tools)
* [ignition-math](http://ignitionrobotics.org/libraries/math)
* [ignition-msgs](http://ignitionrobotics.org/libraries/messages)
* [ignition-transport](http://ignitionrobotics.org/libraries/transport)

[[file:files/gazebo_dependency_tree.svg|400px]]

## A bit of history

All the libraries listed here are evolutions of libraries which were at some
point built within the Gazebo project itself. In an effort to make these
available for other projects and to make Gazebo more modular, they have been
extracted from Gazebo.

### SDFormat

#### SDF protocol

Gazebo uses the Simulation Description Format (SDF) protocol to describe every
aspect of simulation. The SDF protocol is based on XML, you can take a look at
its specification [here](http://sdformat.org/spec). The protocol consists of a
series of (*.sdf) files.

Current protocol versions available are 1.4, 1.5 and 1.6.

#### SDFormat C++ library

Gazebo uses the SDFormat C++ library to parse the SDF protocol.

> Both the SDF protocol and the SDFormat C++ parser are hosted in the same
> repository and will be installed at the same time when performing an
> installation from source.

> Please note that SDFormat library versions follow a semantic versioning where
> major versions correspond to changes in ABI. Its versioning scheme has nothing
> to do with the SDF protocol supported.

#### Versions

Gazebo has had a dependency on the SDFormat library (which automatically handles
the SDF protocol supported) since early versions:

* Gazebo 1.9 - SDFormat > 1? (SDF protocol <= 1.4)
* Gazebo 2.2 - SDFormat > 1.4.7 and < 2.0 (SDF protocol <=  1.5)
* Gazebo 3 - SDFormat > 2.0.1 and < 3.0 (SDF protocol <=  1.5)
* Gazebo 4 - SDFormat > 2.0.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo 5 - SDFormat > 2.3.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo 6 - SDFormat > 3.1.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo 7 - SDFormat > 4.1.0 and < 5.0 (SDF protocol <=  1.6)
* Gazebo 8 - SDFormat 5.0 (SDF protocol <=  1.6)
* Gazebo 9 - SDFormat 6.0 (SDF protocol <= 1.6)

### Ignition Common

* Gazebo 9 - Ignition common 1

### Ignition Fuel Tools

* Gazebo 9 - Ignition fuel tools 1

### Ignition Math

Gazebo has a dependency on Ignition Math from version 6.

* Gazebo 6 - Ignition math 2.0
* Gazebo 7 - Ignition math 2.4
* Gazebo 8 - Ignition math 3 - The built-in gazebo::math library is completely deprecated.
* Gazebo 9 - Ignition math 4 - The built-in gazebo::math library is completely removed.

### Ignition Transport

Gazebo has a dependency on Ignition Transport from version 7.

* Gazebo 7 - Ignition transport 1 or 2
* Gazebo 8 - Ignition transport 3
* Gazebo 9 - Ignition transport 4

### Ignition Messages

Gazebo has a dependency on Ignition Messages from version 8.

* Gazebo 8 - Ignition msgs 0.4
* Gazebo 9 - Ignition msgs 1.0

## Remove packages to get a clean system

Be sure of removing any distribution package of the dependencies listed in this
document to get started from a clean system. For example, in .deb distributions
like Debian or Ubuntu this can easily done by running:

        sudo apt-get remove '.*sdformat.*' '.*ignition-.*'

## Build and install Ignition CMake from source

Many of the ignition packages are using the ignition cmake library.

1. Install required dependencies:

        sudo apt-get install build-essential cmake pkg-config

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-cmake /tmp/ign-cmake
        cd /tmp/ign-cmake

1. Create a build directory and go there. Configure the build:

        mkdir build
        cd build
        cmake ../

    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

1. Build and install:

        make -j4
        sudo make install


## Build and install Ignition Math from source

SDFormat, Ignition Messages and Gazebo depend on the Ignition Math library.

1. Install required dependencies:

        sudo apt-get install build-essential \
                             cmake \
                             mercurial \
                             python

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-math /tmp/ign-math
        cd /tmp/ign-math

     **Note:** the `default` branch is the development branch where
     you'll find the bleeding edge code, your cloned repository should be on
     this branch by default but we recommend you switch to the `ign-math4`
     branch if you desire more stability (with the `hg up ign-math4` command).

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install

## Build and install Ignition Common from source

Gazebo and Ignition Fuel Tools depend on the Ignition Common library.

1. Install required dependencies (note that ignition-cmake and ignition-math are out):

        sudo apt-get install build-essential \
                             cmake \
			     libfreeimage-dev \
			     libtinyxml2-dev \
			     uuid-dev \
			     libgts-dev \
			     libavdevice-dev \
			     libavformat-dev \
			     libavcodec-dev \
			     libswscale-dev \
			     libavutil-dev \
			     libprotoc-dev \
	 	             libprotobuf-dev

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-common /tmp/ign-math
        cd /tmp/ign-common

     **Note:** the `default` branch is the development branch where
     you'll find the bleeding edge code, your cloned repository should be on
     this branch by default but we recommend you switch to the `ign-common1`
     branch if you desire more stability (with the `hg up ign-common1` command).

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install

## Build and install SDFormat from source

Gazebo depends on the SDFormat package.

1. Install required dependencies (note that ign-math was left out):

        sudo apt-get install build-essential \
                             cmake \
                             mercurial \
                             python \
                             libboost-system-dev \
                             libtinyxml-dev \
                             libxml2-utils
                             ruby-dev \
                             ruby

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/osrf/sdformat /tmp/sdformat
        cd /tmp/sdformat

     **Note:** the `default` branch is the development branch where you'll find
     the bleeding edge code, your cloned repository should be on this branch by
     default but we recommend you switch to branch `sdf6` if you desire more
     stability

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install

## Build and install Ignition Messages from source

Gazebo and Ignition Transport depend on the Ignition Messages package.

1. Install required dependencies:

        sudo apt-get install build-essential \
                             cmake \
                             mercurial \
                             libprotoc-dev \
                             libprotobuf-dev \
                             protobuf-compiler

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-msgs /tmp/ign-msgs
        cd /tmp/ign-msgs

     **Note:** Ignition messages hasn't released version 1.0 yet. You can use
     the `default` branch for version 0.

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../

    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install


## Build and install Ignition Fuel Tools

Gazebo depends optionally in the Ignition Fuel Tools

1. Install required dependencies (note that ignition-cmake and ignition-common are out):

        sudo apt-get install build-essential \
                             cmake \
			     libzip-dev \
			     libjsoncpp-dev \
			     libcurl4-openssl-dev \
			     libyaml-dev

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-common /tmp/ign-fuel-tools
        cd /tmp/ign-fuel-tools

     **Note:** the `default` branch is the development branch where
     you'll find the bleeding edge code, your cloned repository should be on
     this branch by default but we recommend you switch to the `ign-fuel-tools1`
     branch if you desire more stability (with the `hg up ign-fuel-tools1` command).

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install


## Build and install Ignition Transport from source

Gazebo depends on the Ignition Transport package.

Please follow the instructions on the Ignition Transport
[documents](http://ignition-transport.readthedocs.io/en/latest/installation/installation.html#install-from-sources-ubuntu-linux).

When installing dependencies, make sure you only install the
`libignition-msgs-dev` package if you haven't installed Ignition Messages from
source.
