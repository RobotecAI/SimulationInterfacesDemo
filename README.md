# SimulationInterfacesDemo

## About

This is an O3DE project to demonstrate the usage of [Simulation Interfaces](https://github.com/ros-simulation/simulation_interfaces), a common simulation standard developed by [Robotec AI](https://robotec.ai) and [Open Robotics](https://openrobotics.org) to enable interoperability between different robotic simulators. This project was used during the [ROSCon 2025](https://roscon.ros.org/2025/) talk **TODO: Add talk title and link**.

Additionally, the project contains two standard O3DE robotic templates: `Ros2ProjectTemplate` and `Ros2RoboticManipulationTemplate`, which can be used for other experiments.

> **Note:** This project runs on Linux only, as O3DE ROS 2 integration is currently supported only on Linux.

<img src="docs/project.png" alt="Project overview" width="50%">

## Repository Structure

The repository is organized as follows:
- `Examples`: contains sample ROS 2 scripts to interact with the simulation.
- `Gems`: placeholder for O3DE Gems pulled as submodules.
- `Project`: contains the O3DE project files.
- `ros2_ws`: contains a ROS 2 workspace with some additional packages.
- `Scripts`: placeholder for the script used during the ROSCon talk pulled as a submodule. **TODO: add submodule when ready**

## Project Setup

Follow the instructions in [INSTALL.md](./docs/install.md) to set up and build the project.

## Run the project:

You can find the instructions to run the project in different scenarios and configuration in [RUN.md](./docs/run.md).

## Detailed Explanation of the repository components

### O3DE Project Structure

The O3DE project was generated using the `Ros2RoboticManipulationTemplate` template using O3DE 25.05.1. Next, the project was modified as follows:
- The Warehouse level based on the [Isaac Sim assets](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_environments.html#warehouse) (`warehouse_with_forklifts.usd`) was added as the main scene.
- The Dingo D robot was imported from the [Dingo ROS 2 Description package](https://github.com/dingo-cpr/dingo).
- The Lidar sensor was added to the Dingo D robot to make it usable in navigation scenarios; lights were added to the robot for better look.
- The content of `Ros2ProjectTemplate` was merged into the project to add another demonstrator scene (3 scenes in total).
- The UR10 robot was added to the simulation; the robot was modified to include a gripper from Panda Franka Emika robot.

### Importing assets

The project started with a Warehouse scene from [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_environments.html#warehouse) already in GLB format. The following steps were taken to import the scene into O3DE:
1. The GLB files were converted to GLTF format using `gltf-pipeline` tool (O3DE does not support GLB textures import natively).
2. The GLTF files were imported into Blender for adjustments of materials.
3. The adjusted GLTF files were imported into O3DE Editor and native O3DE material files were created manually.
4. (optional) The colliders were added manually in O3DE Editor.
5. (optional) Additional entity to fix the pivot point position (located outside the objects in the original files) was added.
6. The scene/objects was saved in O3DE native format.

The Dingo D assets can be directly imported into O3DE from the URDF files using the O3DE ROS 2 Robot Importer tool. The only modification made to the original files was the addition of native O3DE materials to improve the appearance of the robot. The visual quality of the imported robot is not as good as the Warehouse scene, as the original assets are low-poly and do not contain high-quality textures.

### ROS 2 Workspace

The workspace is build based on existing resources with some modifications to add a gripper to the UR robot and to make Dingo package work with ROS2 Humble. In particular:
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
    - The `dingo_description` package was modified to work with ROS 2 Humble

> **Note:** the `dingo_description` package was used to import the robot into O3DE, and it was stored in the native format after the import was completed. The package is not needed anymore. It is a part of the workspace to allow tracking of changes made to the original files.
