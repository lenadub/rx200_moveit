# rx200_moveit

## Build

From the workspace folder, run:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run

To launch all three nodes together with Interbotix and MoveIt:

**Fake hardware:**

```bash
ros2 launch robot_control robot.launch.py hardware_type:=fake
```

**Actual robot:**

```bash
ros2 launch robot_control robot.launch.py hardware_type:=actual
```

## Run nodes separately

First, launch Interbotix and MoveIt in a separate terminal:

```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=fake
```

Change `fake` to `actual` when using the physical robot.

**Open a new terminal for each node.** In every new terminal, go to the project workspace folder and run:

```bash
source install/setup.bash
```

**Terminal 1 — Input Manager**

```bash
ros2 run rx200_moveit_control input_manager_node
```

**Terminal 2 — Action Client**

```bash
ros2 run rx200_moveit_control action_client
```

**Terminal 3 — GUI**

```bash
ros2 run rx200_moveit_control rx200_gui
```

Keep all terminals open while using the project.

## Usage

Enter the pick and place coordinates in the GUI and submit the request.
