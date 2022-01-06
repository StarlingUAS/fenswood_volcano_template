"""
Very simple image processor based on example from
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

    def start(self):
        # set up subscriber for image
        state_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

    # on receiving status message, save it to global
    def image_callback(self,msg):
        self.get_logger().info('Got an image')
                    

def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    rclpy.spin(image_node)


if __name__ == '__main__':
    main()