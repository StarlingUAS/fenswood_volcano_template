"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy                                                    # type: ignore

from mavros_msgs.msg import State                               # type: ignore
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore
from geographic_msgs.msg import GeoPoseStamped

g_node = None
last_state = None
last_pos = None
last_alt_rel = None
init_alt = None


def state_callback(msg):
    global last_state
    last_state = msg
    g_node.get_logger().info('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))


def position_callback(msg):
    global last_pos, last_alt_rel
    # determine altitude relative to arm
    if init_alt:
        last_alt_rel = msg.altitude - init_alt
    last_pos = msg
    g_node.get_logger().info('Drone at {}N,{}E altitude {}m'.format(msg.latitude,msg.longitude,last_alt_rel))


def wait_for_new_status():
    """
    Wait for new state message to be received.  These are sent
    at 1Hz so is equivalent to one second delay.
    """
    if last_state:
        # if had a message before, wait for higher timestamp
        last_stamp = last_state.header.stamp.sec
        for try_wait in range(60):
            rclpy.spin_once(g_node)
            if last_state.header.stamp.sec > last_stamp:
                break
    else:
        # if never had a message, just wait for first one          
        for try_wait in range(60):
            if last_state:
                break
            rclpy.spin_once(g_node)


def request_datastream(msg_id, time_interval):
    cli = g_node.create_client(CommandLong, 'mavros/cmd/command')
    req = CommandLong.Request()
    req.command = 511
    req.param1 = float(msg_id)
    req.param2 = float(time_interval)
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('command_int service not available, waiting again...')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Requested message {} at interval {}.'.format(msg_id, time_interval))


def request_mode_change(new_mode):
    cli = g_node.create_client(SetMode, 'mavros/set_mode')
    req = SetMode.Request()
    req.custom_mode = new_mode
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('set_mode service not available, waiting again...')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Request sent for {} mode.'.format(new_mode))


def request_arm():
    cli = g_node.create_client(CommandBool, 'mavros/cmd/arming')
    req = CommandBool.Request()
    req.value = True
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('arming service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Arming request sent.')


def request_takeoff(target_alt):
    cli = g_node.create_client(CommandTOL, 'mavros/cmd/takeoff')
    req = CommandTOL.Request()
    req.altitude = target_alt
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('takeoff service not available, waiting again...')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Takeoff request sent.')


def main(args=None):
    global g_node, init_alt
    
    rclpy.init(args=args)

    g_node = rclpy.create_node('example_controller')

    # set up two subscribers, one for vehicle state...
    state_sub = g_node.create_subscription(State, 'mavros/state', state_callback, 10)
    state_sub  # prevent unused variable warning

    # ...and the other for global position
    pos_sub = g_node.create_subscription(NavSatFix, 'mavros/global_position/global', position_callback, 10)
    pos_sub  # prevent unused variable warning

    # first wait for the system status to become 3 "standby"
    # see https://mavlink.io/en/messages/common.html#MAV_STATE
    for try_standby in range(60):
        wait_for_new_status()
        if last_state.system_status==3:
            g_node.get_logger().info('Drone ready for flight')
            break 

    # position isn't sent unless we ask for it 
    request_datastream(33,1000000) # msg 33 at 1Hz please

    # now change mode to GUIDED
    # always seems to work so not verified
    request_mode_change("GUIDED")
    
    # next, try to arm the drone
    # keep trying until arming detected in stae message
    for try_arm in range(60):
        request_arm()
        wait_for_new_status()
        if last_state.armed:
            g_node.get_logger().info('Arming successful')
            # armed - grab init alt for relative working
            if last_pos:
                init_alt = last_pos.altitude
            break
    # note the timeout case is not properly handled

    # take off and climb to 20.0m at current location
    request_takeoff(20.0)

    # wait for drone to reach desired altitude
    for try_alt in range(60):
        wait_for_new_status()
        if last_alt_rel > 19.0:
            g_node.get_logger().info('Close enough to flight altitude')
            break

    # move drone by sending setpoint message
    target_pub = g_node.create_publisher(GeoPoseStamped, 'mavros/setpoint_position/global', 10)
    target_msg = GeoPoseStamped()
    target_msg.pose.position.latitude = 51.423
    target_msg.pose.position.longitude = -2.671
    target_msg.pose.position.altitude = init_alt + 20.0 - 50.0 # unexplained correction factor
    target_pub.publish(target_msg)
    g_node.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(target_msg.pose.position.latitude,
                                                                           target_msg.pose.position.longitude,
                                                                           target_msg.pose.position.altitude)) 

    # wait for drone to reach desired position
    for try_arrive in range(60):
        wait_for_new_status()
        d_lon = last_pos.longitude - target_msg.pose.position.longitude
        d_lat = last_pos.latitude - target_msg.pose.position.latitude
        if abs(d_lon) < 0.0001:
            if abs(d_lat) < 0.0001:
                g_node.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                break

    # return home and land
    request_mode_change("RTL")

    while rclpy.ok():
        rclpy.spin_once(g_node)


if __name__ == '__main__':
    main()