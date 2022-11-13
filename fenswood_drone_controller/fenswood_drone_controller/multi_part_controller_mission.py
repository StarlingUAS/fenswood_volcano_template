import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong

from .multi_part_controller_drone_only import DroneController

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('controller')

        # Create object representing drone
        self.drone = DroneController(self)

        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()
        # initial state for finite state machine
        self.control_state = 'init'
        # timer for time spent in each state
        self.state_timer = 0

    def start(self):
        # set up two subscribers, one for vehicle state...
        # ...and the other for global position
        self.drone.start()

        # create a ROS2 timer to run the control actions
        self.timer = self.create_timer(1.0, self.timer_callback)

    def state_transition(self):
        if self.control_state =='init':
            if self.drone.initialised():
                    # move on to arming
                    return('arming')
            else:
                return('init')

        elif self.control_state == 'arming':
            if self.drone.last_status.armed:
                self.get_logger().info('Arming successful')
                # armed - grab init alt for relative working
                if self.drone.last_pos:
                    self.drone.last_alt_rel = 0.0
                    self.drone.init_alt = self.drone.last_pos.altitude
                # send takeoff command
                self.drone.takeoff(20.0)
                return('climbing')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to arm')
                return('exit')
            else:
                self.drone.arm_request()
                return('arming')

        elif self.control_state == 'climbing':
            if self.drone.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                # move drone by sending setpoint message
                self.drone.flyto(51.423, -2.671, self.drone.init_alt - 30.0) # unexplained correction factor on altitude
                return('on_way')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to reach altitude')
                return('landing')
            else:
                self.get_logger().info('Climbing, altitude {}m'.format(self.drone.last_alt_rel))
                return('climbing')

        elif self.control_state == 'on_way':
            d_lon = self.drone.last_pos.longitude - self.drone.last_target.pose.position.longitude
            d_lat = self.drone.last_pos.latitude - self.drone.last_target.pose.position.latitude
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
            self.drone.change_mode("RTL")
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