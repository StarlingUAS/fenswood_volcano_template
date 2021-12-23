"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy                                                    # type: ignore

from mavros_msgs.msg import State                               # type: ignore
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore

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
    global g_node, last_pos, last_alt_rel
    # determine altitude relative to arm
    if init_alt:
        last_alt_rel = msg.altitude - init_alt
    last_pos = msg
    g_node.get_logger().info('Drone at {}N,{}E altitude {}m'.format(msg.latitude,msg.longitude,last_alt_rel))

def wait_for_ready_status():
    # wait for system_status=3 on startup, or exit
    # https://mavlink.io/en/messages/common.html#MAV_STATE
    for ii in range(60):
        rclpy.spin_once(g_node)
        if not rclpy.ok():
            return False
        if last_state:
            if last_state.system_status==3:
                g_node.get_logger().info('Drone ready for flight')
                return True
    # if get to here, timed out
    return False


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


def request_mode_change(new_mode="GUIDED"):
    cli = g_node.create_client(SetMode, 'mavros/set_mode')
    req = SetMode.Request()
    req.custom_mode = new_mode
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('set_mode service not available, waiting again...')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Request sent for {} mode.'.format(new_mode))


def arm_drone():
    global last_state, init_alt
    cli = g_node.create_client(CommandBool, 'mavros/cmd/arming')
    req = CommandBool.Request()
    req.value = True
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('arming service not available, waiting again...')
    # multiple attempts
    for try_arm in range(60):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)
        g_node.get_logger().info('Arming request sent.')
        # wait for next status message
        last_state = None
        while last_state is None:
            rclpy.spin_once(g_node)
        if last_state.armed:
            g_node.get_logger().info('Arming successful')
            # grab current altitude if we have one
            if last_pos:
                init_alt = last_pos.altitude
            break


def request_takeoff(target_alt=20.0):
    cli = g_node.create_client(CommandTOL, 'mavros/cmd/takeoff')
    req = CommandTOL.Request()
    req.altitude = target_alt
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('takeoff service not available, waiting again...')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Takeoff request sent.')


def main(args=None):
    global g_node
    
    rclpy.init(args=args)

    g_node = rclpy.create_node('example_controller')

    state_sub = g_node.create_subscription(State, 'mavros/state', state_callback, 10)
    state_sub  # prevent unused variable warning

    wait_for_ready_status()

    request_mode_change()

    # subscribe to global position and request MAVLINK data at 1Hz
    pos_sub = g_node.create_subscription(NavSatFix, 'mavros/global_position/global', position_callback, 10)
    pos_sub  # prevent unused variable warning
    request_datastream(33,1000000)
    
    arm_drone()

    request_takeoff()

    

    while rclpy.ok():
        rclpy.spin_once(g_node)


if __name__ == '__main__':
    main()