# Old-School Example

[Back to tutorial contents](README.md#contents)

## Introduction

In this tutorial, a simple Python script will be used to control the drone, with the control flow managed just by the Python language flow.  This approached is referred to as 'old school' in the [minimal ROS2 examples](https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber) and is _not_ good programming style.  However, it enables us to focus purely on the ROS interactions, stripped of any Python organization that may be unfamiliar to some readers.  Therefore, the old school way seems the right place to start.

> Style matters.  Good programming style makes code that is re-usable, maintainable, and likely to be correct.  There are some celebrated [lessons of style](https://en.wikipedia.org/wiki/The_Elements_of_Programming_Style#Lessons).  It is also a hotly debated topic in Python, which offers many ways of doing every task, but some more [Pythonic](https://www.udacity.com/blog/2020/09/what-is-pythonic-style.html) than others.

## Threads

ROS uses a [publish/subscribe](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern) model to exchange data between components using named _topics_.  For example, for us to get the drone position, we learn [from the documentation](http://wiki.ros.org/mavros#mavros.2FPlugins.Published_Topics-2) that MAVROS will publish it to a topic named `/vehicle_1/mavros/global_position/global` using the `sensor_msgs/NavSatFix` message type.  Then we must subscribe to that topic to receive those messages.  When we subscribe, we write a _callback_ function that will run every time a position message is received and send its name when we subscribe.

This is a very powerful and scalable way of sharing information *but* it means our program is multi-threaded: different parts of it will run at different times, potentially on top of each other.  Python will handle _most_ of the pain, like worrying what if a message changes halfway through us processing it.  However, some ugliness remains: we will need to use global variables to allow the different threads to communicate with each other.  In our example this will be really simple: the callbacks will just save the messages as they come in, and our code will just look at the last one saved.

> You will need to think about this more when you start working with the camera feed.  You _could_ just put a load of image processing code in a callback, but what happens if you get a new image in while you're still processing the last one?

Our programme will have three threads:
 - the 'main' thread, executing the function `main`
 - the 'state' thread, executing the function `state_callback` whenever a state message is received
 - the `position` thread, executing the function `position_callback` whenever a positon message is received

## Example code

To load this example, first run `git checkout old_school`.  The key file is `controller.py` in the [fenswood_drone_controller/fenswood_drone_controller](../fenswood_drone_controller/fenswood_drone_controller) subdirectory.  The remainder of this section describes how it works.

```
import rclpy
```
The library `rclpy` contains the core functionality for using ROS from Python.
```
# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong
```
ROS communications are all typed, so to use any ROS channels requires importing the relevant information type as a Python class.  There are two types of these definitions: _messages_ from `.msg` libraries and _services_ from `.srv` libraries: more on the distinction later.

Each class is imported from the ROS package that defines it.  For our example, we need:
 - the [State message from the `mavros_msgs` package](http://docs.ros.org/en/api/mavros_msgs/html/msg/State.html) which will convey the readiness status from the drone
 - the [NavSatFix message from the `sensor_msgs` package](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) which will tell us the position from the drone
 - the [GeoPoseStamped message from the `geographic_msgs` package](https://docs.ros.org/en/api/geographic_msgs/html/msg/GeoPoseStamped.html) which will take our position setpoint back to the drone
 - the services SetMode, CommandBool, CommandTOL, CommandLong from the `mavros_msg` package, which we will use to change modes, arm, take-off, and request data streams, respectively.

```
g_node = None           # global for the node handle
g_last_state = None     # global for last received status message
g_last_pos = None       # global for last received position message
g_init_alt = None       # global for global altitude at start
g_last_alt_rel = None   # global for last altitude relative to start
```
Recalling the earlier discussion about [threading](#threads) these global variables will be used to store information from the callback functions for use in the main control thread (or, in the case of the `g_node` variable, _vice versa_).  Initial values of `None` means we can trap cases when no data has been received.

> Global variables are horrible things.  You should really never use them as they can lead to all sorts of unexpected behaviour down the line - but they get us working quickly here.  They will be replaced in the nery next tutorial.

```
def state_callback(msg):
    global g_last_state
    g_last_state = msg
    g_node.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))
```
Above is our first callback function, which will run whenever we get a drone status message.  The `global` line is needed before we can write to the global variable of that name, not just a local copy.  Then all we do is save the incoming data in `msg` to the global and write a log entry.  There are different grades of logging and `debug()` is the lowest, meaning we won't get a screen clogged with these messages unless we go and ask for them.

> We didn't need a `global` declaration for g_node because we're only reading from it.  However, since the function writes to `g_last_state`, a `global g_last_state` is needed.  It's a Python thing.

```
def position_callback(msg):
    global g_last_pos, g_last_alt_rel
    # determine altitude relative to start
    if g_init_alt:
        g_last_alt_rel = msg.altitude - g_init_alt
    g_last_pos = msg
    g_node.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                    msg.longitude,
                                                                    g_last_alt_rel))
```
Above is the other callback function, this time for the position message.  The `if g_init_alt:` clause will only run if `g_init_alt` has been set to something other than its initial value of `None`.  This means the `g_last_alt_rel` calculation only happens if `g_init_alt` has been set.  This is part of a workaround to work with altitude relative to take-off position rather than the ambiguous absolute value, as discussed in the [drone control tutorial](drone_control.md).  `g_init_alt` will be set later on when the drone is armed, after which the callback will also calculate the altitude relative to takeoff and store it in `g_last_alt_rel`.
```
def wait_for_new_status():
    """
    Wait for new state message to be received.  These are sent at
    1Hz so calling this is roughly equivalent to one second delay.
    """
    if g_last_state:
        # if had a message before, wait for higher timestamp
        last_stamp = g_last_state.header.stamp.sec
        for try_wait in range(60):
            rclpy.spin_once(g_node)
            if g_last_state.header.stamp.sec > last_stamp:
                break
    else:
        # if never had a message, just wait for first one          
        for try_wait in range(60):
            if g_last_state:
                break
            rclpy.spin_once(g_node)
```
Our simulation runs in real time and the controller will sometimes need to wait for things to happen.  I could have just used `time.sleep(1)` from the `time` Python library.  
```
def main(args=None):
    global g_node, g_init_alt
    
    rclpy.init(args=args)

    g_node = rclpy.create_node('example_controller')

    # set up two subscribers, one for vehicle state...
    state_sub = g_node.create_subscription(State, 'mavros/state', state_callback, 10)

    # ...and the other for global position
    pos_sub = g_node.create_subscription(NavSatFix, 'mavros/global_position/global', position_callback, 10)

    # first wait for the system status to become 3 "standby"
    # see https://mavlink.io/en/messages/common.html#MAV_STATE
    for try_standby in range(60):
        wait_for_new_status()
        if g_last_state.system_status==3:
            g_node.get_logger().info('Drone ready for flight')
            break 

    # send command to request regular position updates
    cmd_cli = g_node.create_client(CommandLong, 'mavros/cmd/command')
    while not cmd_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('command_int service not available, waiting again...')
    cmd_req = CommandLong.Request()
    cmd_req.command = 511
    cmd_req.param1 = float(33)  # msg ID for position is 33 \
                                # https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
    cmd_req.param2 = float(1000000)    # 1000000 micro-second interval : 1Hz rate
    future = cmd_cli.call_async(cmd_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Requested position stream')

    # now change mode to GUIDED
    mode_cli = g_node.create_client(SetMode, 'mavros/set_mode')
    while not mode_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('set_mode service not available, waiting again...')
    mode_req = SetMode.Request()
    mode_req.custom_mode = "GUIDED"
    future = mode_cli.call_async(mode_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Request sent for GUIDED mode.')
    
    # next, try to arm the drone
    arm_cli = g_node.create_client(CommandBool, 'mavros/cmd/arming')
    while not arm_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('arming service not available, waiting again...')
    # build the request
    arm_req = CommandBool.Request()
    arm_req.value = True
    # keep trying until arming detected in state message, or 60 attempts
    for try_arm in range(60):
        future = arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(g_node, future)
        g_node.get_logger().info('Arming request sent.')
        wait_for_new_status()
        if g_last_state.armed:
            g_node.get_logger().info('Arming successful')
            # armed - grab init alt for relative working
            if g_last_pos:
                g_init_alt = g_last_pos.altitude
            break
    else:
        g_node.get_logger().error('Failed to arm')

    # take off and climb to 20.0m at current location
    takeoff_cli = g_node.create_client(CommandTOL, 'mavros/cmd/takeoff')
    while not takeoff_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('takeoff service not available, waiting again...')
    # build the request
    takeoff_req = CommandTOL.Request()
    takeoff_req.altitude = 20.0
    # only call once - seems to work OK
    future = takeoff_cli.call_async(takeoff_req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Takeoff request sent.')

    # wait for drone to reach desired altitude, or 60 attempts
    for try_alt in range(60):
        wait_for_new_status()
        g_node.get_logger().info('Climbing, altitude {}m'.format(g_last_alt_rel))
        if g_last_alt_rel > 19.0:
            g_node.get_logger().info('Close enough to flight altitude')
            break

    # move drone by sending setpoint message
    target_pub = g_node.create_publisher(GeoPoseStamped, 'mavros/setpoint_position/global', 10)
    wait_for_new_status() # short delay after creating publisher ensures message not lost
    target_msg = GeoPoseStamped()
    target_msg.pose.position.latitude = 51.423
    target_msg.pose.position.longitude = -2.671
    target_msg.pose.position.altitude = g_init_alt - 30.0 # unexplained correction factor
    target_pub.publish(target_msg)
    g_node.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(target_msg.pose.position.latitude,
                                                                           target_msg.pose.position.longitude,
                                                                           target_msg.pose.position.altitude)) 

    # wait for drone to reach desired position, or timeout after 60 attempts
    for try_arrive in range(60):
        wait_for_new_status()
        d_lon = g_last_pos.longitude - target_msg.pose.position.longitude
        d_lat = g_last_pos.latitude - target_msg.pose.position.latitude
        g_node.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
        if abs(d_lon) < 0.0001:
            if abs(d_lat) < 0.0001:
                g_node.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                break

    # return home and land
    mode_req.custom_mode = "RTL"
    future = mode_cli.call_async(mode_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Request sent for RTL mode.')
    
    # now just serve out the time until process killed
    while rclpy.ok():
        rclpy.spin_once(g_node)


if __name__ == '__main__':
    main()
```