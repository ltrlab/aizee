# Open Sauce 2025: AIZEE @ Booth #250372
This repository contains all the code necessary to operate AIZEE for a teleoperation demo at Open Sauce 2025. The workspaces are broken into folders for their respective machines.

## Quick Links
| [Confluence Page](https://ltrlabs.atlassian.net/wiki/x/AYDpC) | [Mechanical Design](https://ltrlabs.atlassian.net/jira/software/projects/HD/boards/1/backlog) | [Software Development](https://ltrlabs.atlassian.net/jira/software/projects/SW/boards/2/backlog) | [Logistics Plan](https://ltrlabs.atlassian.net/jira/software/projects/LOGIC/boards/6/backlog) |

## Quick Setup
- Clone this repository.
```bash
git clone https://github.com/ltrlab/opensauce-demo-2025.git
cd opensauce-demo-2025
./start-demo
```

## Directory Structure
```
opensauce-demo-2025/              ← Root of the repository
├── .github/
│   ├── ISSUE_TEMPLATE.md         ← Bug report & feature request templates
│   └── PULL_REQUEST_TEMPLATE.md  ← Pull request template for Jira linking
│
├── README.md                     ← Landing page
│
├── docs/                         ← Project documentation & assembly
│   ├── ASSEMBLY.md               ← Full assembly guide.
│   └── wiring_diagrams/
│       └── platform_wiring.pdf
│
├── BOM/
│   └── bill_of_materials.xlsx    ← Full assembly guide.
│
├── jetson_ws/                    ← ROS 2 workspace for everything on Jetson
│
├── teensy/                       ← All code that lives on the Teensy 4.1
└── .gitignore                    ← Standard ignores (build artifacts, temp files)
```
## Bill of Materials
The total cost of this robot is roughly $2,000 depending on your region. The complete parts list along with some suppliers for each part lives here:
```
BOM/bill_of_materials.xlsx
```

## Getting Started

1. **Prepare your hardware**
    - 3D-print all parts in design/CAD/exports/ (STL files).
    - Assemble the frame according to docs/ASSEMBLY.md.
    - Wire Jetson Orin Nano Super ↔ Teensy 4.1 using wiring diagrams in docs/wiring\_diagrams/.
    - Install four hoverboard motors onto the mobile platform (see assembly images).
2. **Jetson Software Setup**
    - Install Ubuntu 22.04 LTS on the Jetson Orin Nano Super.
      
    ```bash
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb\_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update && sudo apt install ros-humble-desktop
    ```
    - Clone this repo and build:
      
    ```bash
    cd ~/aizee-household-robot/jetson_ws
    colcon build
    source install/setup.bash
    ```
4. **Teensy Firmware**
    *   Install Teensyduino (latest).
    *   Open teensy/src/aizee\_motors.ino in Arduino IDE.
    *   Select “Teensy 4.1” as the board, compile and upload.
6. **Run the Teleoperation Demo**
   ```bash
   cd ~/aizee-household-robot/jetson_ws
   source install/setup.bash
   ros2 launch aizee_control teleop.launch.py
   ```
   - On Windows (Unity + Oculus):
        1.  Start Unity application.
        2.  Ensure you’re on the same Wi-Fi network as the Jetson.
        3.  Launch the Unity application.
    
## Controls and Operation

_TBD_
