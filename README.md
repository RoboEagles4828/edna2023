# edna2023
2023 FRC Robot

## Requirements
-------
- RTX Enabled GPU

# Workstation Setup steps

### 1. Omniverse Setup

 **Install Nvidia Drivers** \
`sudo apt-get install nvidia-driver-525`

- **Download Omniverse Launcher** \
https://www.nvidia.com/en-us/omniverse/download/

- **Run Omniverse** \
`chmod +x omniverse-launcher-linux.AppImage` \
`./omniverse-launcher-linux.AppImage`

- **Install Cache and Nucleus Server** \
Wait until both are downloaded and installed.

- **DONE with Omniverse Setup**

### 2. Local Setup
- **Install Docker and Nvidia Docker** \
`./scripts/local-setup.sh`

- **Install Remote Development VS Code Extension** \
Go to extensions and search for Remote Development and click install

- **Reopen in Devcontainer** \
Hit F1 and run the command Reopen in Container and wait for the post install to finsih EST: 10-30min

### 3. Isaac Setup

- **Create Shaders** \
This uses a lot of cpu resource and can take up to 20min \
`isaac-setup`

- **Launch Isaac** \
`isaac-setup`

### 4. Build ROS2 Packages

- **Install Workspace Dependencies** \
Press F1, and run this command: \
`ROS: Install ROS Dependencies for this workspace using rosdep`

- **Build the packages** \
Shortcut: `ctrl + shift + b` \
or \
Terminal: `colcon build --symlink-install`

- **Done with Building ROS2 Packagess**

# Running Edna

1. Connect an xbox controller  
2. Hit load on the *Import URDF* extension window
3. Press Play on the left hand side
4. From this repo directory run \
`source install/setup.bash` \
`ros2 launch swerve_description isaac.launch.py`
5. Use the controller to move the robot around