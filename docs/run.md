# Running the SimulationInterfacesDemo Project

## Run the project (ROSCon demo):

This instruction should be used to run the project as demonstrated during the ROSCon 2025 talk. Start the project directly from the build folder:
```shell
cd $PROJECT_PATH/Project
build/linux/bin/profile/SimulationInterfacesDemo.GameLauncher
```

You might want to use some command line arguments to run the simulation full screen, disable some debug prints and select a specific level:
```shell
cd $PROJECT_PATH/Project
build/linux/bin/profile/SimulationInterfacesDemo.GameLauncher -r_fullscreen=1 -bg_ConnectToAssetProcessor=0 -r_displayInfo=0 +LoadLevel=warehouse
```

Next, you need to start the demo script located in `Scripts` folder. Please refer to the [instructions](../Scripts/README.md) in the respective submodule.

## Run the project in O3DE Editor:

Running the project in O3DE Editor allows you to modify the scenes and experiment with different configurations. Depending on your installation, you can start the Editor and load the project either from SDK installation:
```shell
/opt/O3DE/25.05.1/bin/Linux/profile/Default/Editor --project-path $PROJECT_PATH/Project
```
or from the local build of O3DE (when built from source):

```shell
cd $PROJECT_PATH/Project
build/linux/bin/profile/Editor
```

## Run the MoveIt demo for UR10 robot with a gripper:

MoveIt demo with a gripper for UR10 robot can be run using the ROS 2 workspace included in the repository. First, build and source the `ros2_ws` workspace. Run the MoveIt launcher:

```bash
cd $PROJECT_PATH/ros2_ws
colcon build
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true use_fake_hardware:=true use_sim_time:=true
```

## Run Ros2ProjectTemplate scene:
You can run the Ros2ProjectTemplate scene included in the project to test the basic ROS 2 integration features. Start the project from the build folder with the appropriate level loaded:

```shell
cd $PROJECT_PATH/Project
build/linux/bin/profile/SimulationInterfacesDemo.GameLauncher +LoadLevel=demolevel
```

Use the second terminal to run the example ROS 2 scripts located in `Examples` folder:
```shell
cd $PROJECT_PATH/Examples
ros2 launch navigation.launch.py 
```

Note, that this example also shows how to implement the ROS 2 node directly in C++ as an O3DE component. You can find the implementation in `Project/Gem/Source/SampleComponent.cpp`. This piece of code creates a debug buttons in ImGui debug interface to publish simple messages to a ROS 2 topic. The debug interface can be accessed when starting the simulation from the Editor and pressing `Home` key:
- Start the Editor.
- Load the `demolevel`.
- Run the navigation stack from `Examples` folder.
- Press `Ctrl+G` to start the simulation.
- Press `Home` key to open the ImGui debug interface.

## Run Ros2RoboticManipulationTemplate scene:
You can run the Ros2RoboticManipulationTemplate scene included in the project to test robotic manipulation scenario with Panda Franka Emika robot with a gripper. Start the project from the build folder with the appropriate level loaded:

```shell
cd $PROJECT_PATH/Project
build/linux/bin/profile/SimulationInterfacesDemo.GameLauncher +LoadLevel=roboticmanipulation
```

Use the second terminal to run the example ROS 2 scripts located in `Examples` folder:
```shell
cd $PROJECT_PATH/Examples
ros2 launch panda_moveit_config_demo.launch.py
```
This should open RViz with Panda robot model and MoveIt configuration. You can use the `Planning` tab in RViz to plan and execute motions for the robot.

# Troubleshooting
If you encounter any issues while running the project, consider the following troubleshooting steps:
1. Ensure that all environment variables are set correctly, especially `PROJECT_PATH`.
2. Verify that the ROS 2 environment is built and sourced properly.
3. Make sure the simulation is running (Simulation Interfaces standard defines the simulation state _stopped_ as an initial state).
    The simulation can be started by pressing `R` key when the simulation window is focused. It can be also set to get _playing_ state on start by default by changing `StartInStoppedState` property in [../Project/Registry/ros2.setreg](../Project/Registry/ros2.setreg) file to `false`. Finally, you can enable the debug print of the state in the same file by setting `PrintStateNameInGui` property to `true`.
4. Check the O3DE Editor logs for any error messages that might indicate missing assets or configuration issues.
5. Make sure that the Asset Processor has been run at least once to process all assets.
