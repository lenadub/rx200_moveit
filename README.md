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

## Limits 

There are limits to valid coordinates that the system will accept that are handled by the GUI. 
Negative values of z are rejected and the max z height is 0.4m 

Through testing of the physical arm, the max reliable reach was determined to be 0.55m and the minumum was 0.275m 
Coordinates within 0.03m in the same axis (i.e. only change x from 0.4 - 0.37) are treated as the same coordinate 

These limits are in place to prevent potentially unsafe movement of the robot arm and due to the slight error in location that arises during movement
