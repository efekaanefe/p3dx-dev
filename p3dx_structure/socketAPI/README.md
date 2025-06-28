# P3DX API w/o ROS2

This api sends the robot velocity information the user wants via socket. There is a keyboard mode that can be opened and closed. 

The user can either determine the velocity directly, move the robot with keyboard or enter an xy coordinate (assuming robot starts at (0.0)) for the robot to go.

To try this api without a usage of socket, set the is_simulation parameter false at the beginning.

### Prerequisites

`pip install keyboard`

The ROS1 code present under p3dx_structure/rb1. Follow its README for more information on how to set it up.

