# O3DE Project Installation Guide

You may install this project using O3DE SDK or build O3DE from source. 

> **Note:** The project is compatible with O3DE version 25.05.1.

## Installation Steps using O3DE SDK

> **Note:** The instruction based on O3DE SDK installation works correctly only on Ubuntu 22.04 with ROS 2 Humble due to some ABI incompatibilities when running on newer Ubuntu versions. If you wish to use different compiler version or different operating system, you need to build O3DE from source.

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
cd $PROJECT_PATH/Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=ON -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile
```

5. Run the Asset Processor to process the assets:
```shell
cd $PROJECT_PATH/Project
/opt/O3DE/25.05.1/bin/Linux/profile/Default/AssetProcessor --project-path $PROJECT_PATH/Project &
```

> **Note:** The last step is optional but recommended ensuring that all assets are properly processed before running the project for the first time.

## Installation Steps building O3DE from source

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
cd $PROJECT_PATH/Project
sed -i 's/"engine": "o3de-sdk"/"engine": "o3de"/' project.json
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=ON -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile --target SimulationInterfacesDemo SimulationInterfacesDemo.GameLauncher SimulationInterfacesDemo.Assets Editor
```

> **Note:** the build process may take a while, as it needs to compile O3DE engine and all the gems.
