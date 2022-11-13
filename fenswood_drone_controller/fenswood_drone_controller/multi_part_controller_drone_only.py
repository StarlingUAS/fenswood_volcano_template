import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong

class DroneController():

    def __init__(self, node):
        self.node = node            # store the rosnode object
        self.last_status = None     # store for last received status message
        self.last_pos = None       # store for last received position message
        self.init_alt = None       # store for global altitude at start
        self.last_alt_rel = None   # store for last altitude relative to start
        # create service clients for long command (datastream requests)...
        self.cmd_cli = self.node.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
        # ... for mode changes ...
        self.mode_cli = self.node.create_client(SetMode, '/vehicle_1/mavros/set_mode')
        # ... for arming ...
        self.arm_cli = self.node.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
        # ... and for takeoff
        self.takeoff_cli = self.node.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')
        # create publisher for setpoint
        self.target_pub = self.node.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()


    def start(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.node.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.node.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)

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

    def initialise(self):
        if self.drone.last_status and self.drone.last_status.system_status == 3:
            self.get_logger().info('Drone initialized')
            # send command to request regular position updates
            self.request_data_stream(33, 1000000)
            # change mode to GUIDED
            self.drone.change_mode("GUIDED")
            # Drone initialised and attempting to change mode
            return True
        # Drone not yet initialised
        return False
