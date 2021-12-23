"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy                                                    # type: ignore

from mavros_msgs.msg import State                               # type: ignore
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL    # type: ignore

g_node = None
last_state = None

def state_callback(msg):
    global g_node, last_state
    last_state = msg
    g_node.get_logger().info('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))


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
    global last_state
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

    subscription = g_node.create_subscription(State, 'mavros/state', state_callback, 10)
    subscription  # prevent unused variable warning

    wait_for_ready_status()

    request_mode_change()
    
    arm_drone()

    request_takeoff()

    while rclpy.ok():
        rclpy.spin_once(g_node)


if __name__ == '__main__':
    main()