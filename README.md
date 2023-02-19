# edna2023
2023 FRC Robot

### Requirements
-------
- RTX Enabled GPU

# Workstation Setup steps

### 1. Install Graphics Drivers

 **Install Nvidia Drivers** \
`sudo apt-get install nvidia-driver-525`

### 2. Local Setup
- **Install Docker and Nvidia Docker** \
`./scripts/local-setup.sh`

- **Install Remote Development VS Code Extension** \
Go to extensions and search for Remote Development and click install

- **Reopen in Devcontainer** \
Hit F1 and run the command Reopen in Container and wait for the post install to finsih.

### 3. Isaac Container Setup

- **Create Shaders** \
This uses a lot of cpu resource and can take up to 20min \
`isaac-setup`

- **Launch Isaac** \
`isaac`

### 4. Build ROS2 Packages

- **Build the packages** \
Shortcut: `ctrl + shift + b` \
or \
Terminal: `colcon build --symlink-install`

- **Done with Building ROS2 Packagess**

### 5. (Optional) Omniverse Setup

- **Download Omniverse Launcher** \
https://www.nvidia.com/en-us/omniverse/download/

- **Run Omniverse** \
`chmod +x omniverse-launcher-linux.AppImage` \
`./omniverse-launcher-linux.AppImage`

- **Install Cache and Nucleus Server** \
Wait until both are downloaded and installed.

- **DONE with Omniverse Setup**
# Running Edna

### Inside of Isaac

1. Connect an xbox controller
2. Open Isaac
3. Open Isaac and hit load on the *Import URDF* extension window
4. Press Play on the left hand side
5. Run `launch isaac` inside devcontainer

### In real life (TODO)