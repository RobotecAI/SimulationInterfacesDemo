# SimulationInterfacesDemo

## Setup

You may install this project using O3DE SDK or build O3DE from source. The instruction based on O3DE SDK installation was tested on Ubuntu 22.04 with ROS 2 Humble, and it requires Clang 12. If you wish to use different compiler version or different operating system, you need to build O3DE from source.

### Installation Steps using O3DE SDK

1. Install and initialize O3DE:
```shell
wget https://o3debinaries.org/main/Latest/Linux/o3de_2505_1.deb
sudo dpkg -i o3de_2505_1.deb
/opt/O3DE/25.05.1/python/get_python.sh
/opt/O3DE/25.05.1/scripts/o3de.sh register --this-engine 
```

2. Set the environment variable pointing to this repository:
```shell
export PROJECT_PATH="/path/to/SimulationInterfacesDemo"
```

3. Initialize submodules:
```shell
cd $PROJECT_PATH
git submodule update --init --recursive
```

3. Register gems:
```shell
cd $PROJECT_PATH
/opt/O3DE/25.05.1/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2/
/opt/O3DE/25.05.1/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2SampleRobots/
/opt/O3DE/25.05.1/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/LevelGeoreferencing/
/opt/O3DE/25.05.1/scripts/o3de.sh register -gp ./Gems/o3de-ur-robots-gem/
```
4. Build and source the `ros2_ws` workspace:
```bash
cd $PROJECT_PATH
cd ros2_ws
colcon build
source install/setup.bash
```

5. Build the project:
```shell
cd $PROJECT_PATH
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=ON -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile
```

### Installation Steps building O3DE from source

1. Set the environment variable pointing to this repository and to O3DE source code:
```shell
export PROJECT_PATH="/path/to/SimulationInterfacesDemo"
export O3DE_PATH="/path/to/o3de"
```

2. Clone the O3DE repository, checkout the desired version and initialize O3DE:
```shell
git clone https://github.com/o3de/o3de.git $O3DE_PATH
cd $O3DE_PATH
git checkout 2505.1
./python/get_python.sh
./scripts/o3de.sh register --this-engine
```

3. Initialize submodules:
```shell
cd $PROJECT_PATH
git submodule update --init --recursive
```

4. Register gems:
```shell
cd $PROJECT_PATH
$O3DE_PATH/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2/
$O3DE_PATH/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2SampleRobots/
$O3DE_PATH/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/LevelGeoreferencing/
```

5. Build and source the `ros2_ws` workspace:
```bash
cd $PROJECT_PATH
cd ros2_ws
colcon build
source install/setup.bash
```

6. Build the project:
```shell
cd $PROJECT_PATH
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=ON -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile
```

> **Note:** the build process may take a while, as it needs to compile O3DE engine and all the gems.

## Run the project:

You can start the project directly from the build folder:
```shell
cd $PROJECT_PATH/Project
build/linux/bin/profile/SimulationInterfacesDemo.GameLauncher
```

or from the O3DE Editor:
```shell
/opt/O3DE/25.05.1/bin/Linux/profile/Default/Editor --project-path ~/devroot/projects/SimulationInterfacesDemo/Project 
```
