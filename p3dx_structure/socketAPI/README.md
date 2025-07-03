# P3DX API w/o ROS2

This api sends the robot velocity information the user wants via socket. There is a keyboard mode that can be opened and closed. 

The user can either determine the velocity directly, move the robot with keyboard or enter an xy coordinate (assuming robot starts at (0.0)) for the robot to go.

To try this api without a usage of socket, set the is_simulation parameter false at the beginning.

## Prerequisites

If you want this to be a simulation, no need for ros1. But if not, then follow the ros1 tutorial in the readme under the folder p3dx_structure.

Also, you will need to install a package for the keyboard controls:
```commandline
pip install pynput
```


## Using the API

### Initialization
To use this api within your python script, initialize the class once by calling

```python
robot = p3dxRobot(is_simulation=True)
```

After that, as long as the program is running and (if you chose not to be a simulation) if we are connected to the server socket, the gui will show and it will be possible to interact with it.

### GUI

The gui on the left, will show the movement and the location of the robot. It is possible to arrange the scale it will show by changing the 
```python
self.scale = 400
```
attribute of the class.

On the right, we have some information about the robot:
- Mode: 
    - idle: no commands are given, not in manual mode
    - manual: keyboard controls are enabled
    - autonomus: keyboard is disabled, a command such as go to was given
- Position: position of the robot in the cartesian format
- Angle: Angle of the robot in terms of degrees. When it faces the positive x axis, the angle is 0.
- Velocity: Linear and angular velocity of the robot. 
- Switch to auto/manual mode: Calss stopKeyboard() or useKeyboard() functions accordingly.
- Go to (x,y): Calls the go(x,y) function in a new thread.
- Add obstacle (x,y): Adds obstacles, useful if using obstacle avoidance/lidar. No obstacle avoidance feature is implemented within this api.
- Set speed (linear,angular): Calls the set_velocity function.
- Stop: for emergency stops


### Functions

These are the building block functions that can be called in order to command the robot. All of these functions have self explanatory names.

- add_one_obstacle(obstacle): obstacle is in terms of (x,y)
- add_obstacles([obstacle])
- stop()
- set_velocity(linear_x, angular_z)
- go(target_x, target_y)
- useKeyboard()
- stopKeyboard()

Disconnecting the socket:
- close()





