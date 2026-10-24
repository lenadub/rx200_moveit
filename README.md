# rx200_moveit

## Build

From the workspace folder, run:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run

To launch all three nodes together:

```bash
ros2 launch robot_control rx200.launch.py
```

## Run nodes separately

**Open a new terminal for each node.** In every new terminal, go to the workspace folder and run:

```bash
source install/setup.bash
```

**Terminal 1 — Input Manager**

```bash
ros2 run rx200_moveit_control input_manager_node
```

**Terminal 2 — Robot Controller**

```bash
ros2 run rx200_moveit_control robot_controller_node
```

**Terminal 3 — GUI**

```bash
ros2 run rx200_moveit_control gui
```

Keep all three terminals open while using the project.

## Usage

Enter the pick and place coordinates in the GUI and submit the request.
