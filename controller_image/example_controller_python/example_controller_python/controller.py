"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

g_node = None
last_state = None

def state_callback(msg):
    global g_node, last_state
    last_state = msg
    g_node.get_logger().info('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))


def main(args=None):
    global g_node, last_state
    rclpy.init(args=args)

    g_node = rclpy.create_node('example_controller')

    subscription = g_node.create_subscription(State, 'mavros/state', state_callback, 10)
    subscription  # prevent unused variable warning

    # wait for system_status=3 on startup, or exit
    # https://mavlink.io/en/messages/common.html#MAV_STATE
    for ii in range(60):
        rclpy.spin_once(g_node)
        if not rclpy.ok():
            return
        if last_state:
            if last_state.system_status==3:
                g_node.get_logger().info('Drone ready for flight')
                break

    # request GUIDED mode
    cli = g_node.create_client(SetMode, 'mavros/set_mode')
    req = SetMode.Request()
    req.custom_mode = "GUIDED"
    while not cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('set_mode service not available, waiting again...')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Request sent for GUIDED mode.')
    
    # arm the drone
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

    while rclpy.ok():
        rclpy.spin_once(g_node)


if __name__ == '__main__':
    main()