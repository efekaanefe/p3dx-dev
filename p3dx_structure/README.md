# Running P3DX

This is the guide to run the whole p3dx package with ros2 and the p3dx itself. For the API README, check under the folder socketAPI.

Before everything, determine a common wi-fi and connect every device to that wi-fi.

Also, the raspberry may get heated after connecting it to power and running it for a while, this will show itself as frozen terminals. The simplest way to cut the power and wait for it to cool down.

## Master Script

To get more idea about what to do within the terminals, you can check the master script under the folder p3dx_structure/Master Script. It shows how to run the ros1 section, the obstacle detection and the lidar (realsense camera) and the remaining base nodes (e.g.: rb2_ros2)


## Step 1: ROS1

### Using Raspberry

Note: After learning the IP of the raspberry by pasting the following line to the terminal,
```commandline
hostname -I
```
you can then ssh to the raspberry with:
```commandline
ssh ubuntu@HOST_IP
```
Where the name before the @ sign should match the username of the device. For our raspberry it is `ubuntu`. You can use tmux or ssh 3 times for the following 3 codes we will run.

This is already done automatically within the masterscript specifically for our raspberry.

---

1. Run ros1 on a device connected to the p3dx and in a terminal:
```commandline
roscore
```

2. Open another terminal and paste the following:
```commandline
rosrun rosaria RosAria
```
3. Open a third terminal and paste the following:
```commandline
rosrun pioneer rb1_ros1.py
```
This opens the socket. Since the socket timeouts if there is no connection after some time, you may need to stop and rerun this line time to time.


## Step 2: ROS2

### Set up your ros2 environment

1. Source ros with your version of ros. Since the lidar requires ros2 foxy, we have been using that version.
```commandline
source /opt/ros/foxy/setup.bash
```

2. Create your workspace.

```commandline
cd ~
mkdir -p p3dx_ws/src
```

3. Auto-Generate your package. We will be naming the package rb2
```commandline
cd p3dx_ws/src
ros2 pkg create --build-type ament_python rb2 --dependencies rclpy std_msgs
```

4. Move your source codes into the directory `p3dx_ws/src/rb2/rb2/` . For our case, move all the files under `p3dx_structure/rb2` in the github repository.


5. To be able to run the scripts, open the `setup.py` file under your package folder. Make sure the console scripts list under the entry points dictionary 
looks like this:
```python
        license='MIT',
    entry_points={
        'console_scripts': [
            'rb2_ros2 = rb2_pkg.rb2_ros2:main',
            'aruco_robot = rb2_pkg.velocity_changing_nodes.aruco_robot:main',
            'keyboard_control = rb2_pkg.velocity_changing_nodes.keyboard_control:main',
            'obstacle_detection = rb2_pkg.velocity_changing_nodes.obstacle_detection:main',
            'potential_field = rb2_pkg.velocity_changing_nodes.potential_field:main',
            'target = rb2_pkg.velocity_changing_nodes.target:main',
        ],
    },
```

6. Go back to the root directory of the workspace. Build the workspace there and source it after.
```commandline
cd ..
colcon build
source install/setup.bash
```

Your workspace is now ready!


### Nodes

Currently we have a core script named rb2_ros2.py which opens the socket and connects to the rb1_ros1.py code running within the raspberry. 

Run rb2_ros2 first to connect to the socket:
```commandline
ros2 run rb2 rb2_ros2
```
It will ask you whether you want this to be a simulation or not. If you type `y`, it will not open a socket and will not connect to the raspberry. Type anything else to open the socket.

You can run the other scripts by changing the console cript name in the command above.

Other than the main node, remaining nodes within this package are all for publishing velocity, therefore named "velocity changing nodes"


## Development

You can add different nodes to publish velocity by adding new python files under the package and adding them to the console_scripts.

After that, you need to determine how to add the new velocity component. 
You can decide that by writing a new function within the RobotController class in the pb2_pos2 file. An example where we sum all the incoming velocities is below:
```python
# enumerations to make things easier
KEYBOARD = 0
ARUCO = 1
OBSTACLE = 2
# add your new source here

# the class
class RobotController(Node):
    def __init__(...):
        # other stuff
        
        # subscribe
        self.subscriber_keyboard = self.create_subscription(Twist,'keyboard_vel',self.keyboard_vel_callback,10)
        # subscribe to your new source topic here
        
        self.source_velocities = {
                KEYBOARD: (0.0, 0.0),
                # add your new source here
            }
        
        # set the example function as the current mode function.
        self.curr_mode_func = self.mode_base

    # other functions...   
        
    def mode_base(self):
        """
        example function
        sum the velocities however you like
        Example: you can implement potential field here
        """
        total_linear = sum(v[0] for v in self.source_velocities.values())
        total_angular = sum(v[1] for v in self.source_velocities.values())
        self.set_velocity(total_linear, total_angular)

    # other functions...
    
    def keyboard_vel_callback(self, msg):
        """
        define a new callback for your new velocity source
        as an example, keyboard callback is given
        """
        self.get_logger().info("Received keyboard velocity")
        self.base_callback(msg.linear.x, msg.angular.z, KEYBOARD)

```