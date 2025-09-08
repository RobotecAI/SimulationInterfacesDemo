# SimulationInterfacesDemo

## Setup
0. Set an enviroment variable pointing to this repository:
```bash
export PROJECT_PATH="/path/to/SimulationInterfacesDemo"
```
1. Init submodules:
```bash
cd $PROJECT_PATH
git submodule update --init --recursive
```
2. Get python and register the project:
```bash
cd $PROJECT_PATH
./o3de/scripts/python/get_python.sh
./o3de/scripts/o3de.sh register --this-engine
```

3. Register gems:
```bash
cd $PROJECT_PATH
./o3de/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2/
./o3de/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2SampleRobots/
./o3de/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/LevelGeoreferencing/
./o3de/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2Controllers/
./o3de/scripts/o3de.sh register -gp ./Gems/o3de-extras/Gems/ROS2Sensors
```

4. Build the project:
```bash
cd $PROJECT_PATH
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=OFF -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile
```
