# Using a ROS timer

[Back to tutorial contents](README.md#contents)

## Introduction

The controller is now implemented in its own class, a child of the ROS Node class so it's easy to access ROS functionality.  Basic drone interactions like mode changes and movement commands are implemented as methods containing the necessary ROS functions.  Callback functions record incoming data to properties for other methods to access.  A [finite state machine](finite_state.md#introduction) manages the decision-making.  In this final enhancement, a ROS timer is used to update the finite state machine, making our node in line with established ROS2 best practice.

[Back to tutorial contents](README.md#contents)

## Example code

```
import rclpy                                                    # type: ignore
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State                               # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped                  # type: ignore

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore
```
Imports include `rclpy` for ROS functions, especially the `Node` class to be the parent for our controller class.  The more specific imports represent the message type for every ROS topic and service that the controller will use.
```

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('example_controller')
        self.last_status = None     # store for last received status message
        self.last_pos = None       # store for last received position message
        self.init_alt = None       # store for global altitude at start
        self.last_alt_rel = None   # store for last altitude relative to start
        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()
```
The contructor `__init__` calls the parent constructor to initialize the ROS connections.  Properties for shared information are created and (mostly) initialized to `None` for easy detection of missing data.  
```
        # create service clients for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, 'mavros/cmd/command')
        # ... for mode changes ...
        self.mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        # ... for arming ...
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        # ... and for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        # create publisher for setpoint
        self.target_pub = self.create_publisher(GeoPoseStamped, 'mavros/setpoint_position/global', 10)
```
Clients for service calls and target publishing are all created at initialization.  Subscribers wait for later though as they start the callbacks running.

Note there are no `wait_for_service` or `spin` calls here.  Using the ROS timer handles all these timing elements for us, and deadlocks can result if extra timing calls are included.
```
        # initial state for finite state machine
        self.control_state = 'init'
        # timer for time spent in each state
        self.state_timer = 0
```
The finite state machine and the state step counter are initialized.
```
    def start(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.position_callback, 10)
        # create a ROS2 timer to run the control actions
        self.timer = self.create_timer(1.0, self.timer_callback)
```
A separate `start` method creates the two topic subscriptions and the timer.  Callbacks will start running once this method is called.  I like to keep a separate `start` method in cases like this so you can build a controller object and poke at it before it actually does any ROS work.
```
    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_status = msg
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                        msg.longitude,
                                                                        self.last_alt_rel))
```
These are the two callback functions for the drone status and global position topics, respectively.  These will run in their own threads whenever messages are received.
```
    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        self.get_logger().info('Requested msg {} every {} us'.format(msg_id,msg_interval))

    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        self.get_logger().info('Request sent for {} mode.'.format(new_mode))

    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        self.get_logger().info('Arm request sent')

    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        self.get_logger().info('Requested takeoff to {}m'.format(target_alt))

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        self.target_pub.publish(self.last_target)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 

    def state_transition(self):
        if self.control_state =='init':
            if self.last_status:
                if self.last_status.system_status==3:
                    self.get_logger().info('Drone initialized')
                    # send command to request regular position updates
                    self.request_data_stream(33, 1000000)
                    # change mode to GUIDED
                    self.change_mode("GUIDED")
                    # move on to arming
                    return('arming')
                else:
                    return('init')
            else:
                return('init')

        elif self.control_state == 'arming':
            if self.last_status.armed:
                self.get_logger().info('Arming successful')
                # armed - grab init alt for relative working
                if self.last_pos:
                    self.last_alt_rel = 0.0
                    self.init_alt = self.last_pos.altitude
                # send takeoff command
                self.takeoff(20.0)
                return('climbing')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to arm')
                return('exit')
            else:
                self.arm_request()
                return('arming')

        elif self.control_state == 'climbing':
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                # move drone by sending setpoint message
                self.flyto(51.423, -2.671, self.init_alt - 30.0) # unexplained correction factor on altitude
                return('on_way')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to reach altitude')
                return('landing')
            else:
                self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
                return('climbing')

        elif self.control_state == 'on_way':
            d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
                self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                return('landing')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to reach target')
                return('landing')
            else:
                self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
                return('on_way')
            
        elif self.control_state == 'landing':
            # return home and land
            self.change_mode("RTL")
            return('exit')

        elif self.control_state == 'exit':
            # nothing else to do
            return('exit')

    def timer_callback(self):
        new_state = self.state_transition()
        if new_state == self.control_state:
            self.state_timer = self.state_timer + 1
        else:
            self.state_timer = 0
        self.control_state = new_state
        self.get_logger().info('Controller state: {} for {} steps'.format(self.control_state, self.state_timer))
                    

def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.start()
    rclpy.spin(controller_node)


if __name__ == '__main__':
    main()
```
Main and the Python bit at the end remain unchanged.


## Exercises

All exercises work using the `finite_state` code so run `git checkout finite_state` first.

1. Add a second target and fly to it only if you reach the first target within a limited time, or (b) if you fail to reach the first target in time.

2. Add an extra initial target, but make it very far away.  When you don't reach it in a sensible time, divert back to your original target.

3. Add a human 'approve' input.  When you get to altitude, ask the operator (in the log) if they are happy to proceed, and fly to the target only if they approve.  For input, you can define your own ROS topic and have them publish through Foxglove, or re-purpose the tele-op gamepad buttons to publish to a different topic, or just ask the operator to wiggle the camera.  Test all cases you can think of: operator says 'yes' when asked, operator says 'no' when asked, operator says nothing, operator has said 'yes' but before being asked, _etc._

4. Add a human 'pause' input.  Define a button, topic or some other signal, and make the drone stop and hover if the operator requests it.  There are lots of ways of implementing this, including mode changes or extra control logic.  Don't forget you can connect QGroundControl to the simulation via localhost, TCP port 5761, if you want the operator to interact that way.

[Back to tutorial contents](README.md#contents)