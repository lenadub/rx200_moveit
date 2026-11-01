# rx200_moveit

ROS2 project for controlling the Interbotix RX200 robot using MoveIt and a GUI. The GUI sends pick and place coordinates to the input manager, which checks them before forwarding the request to the action client. The action client controls the robot and sends status updates back to the GUI.

## Build

From the workspace folder, source the Interbotix workspace if needed, then run:

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

Once the GUI opens, enter the pick and place coordinates in the corresponding fields:

- **Pick:** X, Y, Z coordinates of the object.
- **Place:** X, Y, Z coordinates of the destination.

Enter the coordinates as numerical values in metres, then click the submit button to send the request.

The input manager checks the coordinates before sending them to the action client. The GUI displays the robot's status during execution.

You can enter new coordinates after the movement is finished, without restarting the nodes or rebuilding the workspace.

## Team Members

- Kian Anthony Curran Mahon
- Léna Isabel Estelle Dubois
- Caelan Pol Chaney Ó Raghaill
