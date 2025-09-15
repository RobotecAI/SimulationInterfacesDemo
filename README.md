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

4. Build the project:
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

4. Build the project (you need to adjust the engine parameter in `project.json` file to switch from O3DE SDK to your O3DE source code):
```shell
cd $PROJECT_PATH
sed -i 's/"engine": "o3de-sdk"/"engine": "o3de"/' project.json
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

## Run the MoveIt demo:

Build and source the `ros2_ws` workspace. Run the MoveIt launcher:
```bash
cd $PROJECT_PATH/ros2_ws
colcon build
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true use_fake_hardware:=true use_sim_time:=true
```

## ROS 2 Workspace

The workspace is build based on existing resources with some modifications to add a gripper to the UR robot and to make it work with ROS2 Humble. In particular:
1. [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) fork used in [ROSCon2023Demo](https://github.com/RobotecAI/ROSCon2023Demo) repository was used as a base (commit `8569a675`). The following modifications were made to add the gripper:
    - The gripper [assets](./ros2_ws/src/Universal_Robots_ROS2_Description/urdf/finger_gripper) were added.
    - The [urdf/ur_macro.xacro](./ros2_ws/src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro) file was modified adding the gripper.
2. [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/f06092e4f32ae1d042459cfaaae96b5c0ea1b21d) repository was used as a base (commit `f06092e4`). The following modifications were made to add the gripper:
    - All packages except `ur_moveit_config` were removed.
    - The [config/controllers.yaml](./ros2_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/controllers.yaml) file was modified adding the gripper.
    - The [srdf/ur_macro.srdf.xacro](./ros2_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/srdf/ur_macro.srdf.xacro) file was modified including appropriate collision disables.
3. [Dingo](https://github.com/dingo-cpr/dingo) repository was used as a base (commit `68806fd`) for the Dingo robot. The following modifications were made to make the robot usable without gazebo in ROS 2 Humble:
    - All packages except `dingo_description` were removed.
    - The `dingo_description` package was modified to remove all references to gazebo and accessories. The files modified are [urdf/dingo-d.urdf.xacro](./ros2_ws/src/dingo/dingo_description/urdf/dingo-d.urdf.xacro) and [urdf/dingo-o.urdf.xacro](./ros2_ws/src/dingo/dingo_description/urdf/dingo-o.urdf.xacro).
    - The `dingo_description` package was modified to work with ROS2 Humble

> **Note:** the `dingo_description` package was used to import the robot into O3DE, and it was stored in the native format after the import was completed. The package is not needed anymore. It is a part of the workspace to allow tracking of changes made to the original files.
