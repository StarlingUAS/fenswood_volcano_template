"""
Very simple image processor based on example from
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        self.br = CvBridge()

    def start(self):
        # set up subscriber for image
        state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)

    # on receiving image, convert and log information
    def image_callback(self,msg):
        img = self.br.imgmsg_to_cv2(msg)
        # can do OpenCV stuff on img now
        shp = img.shape # just get the size
        self.get_logger().info('Got an image of {} x {}'.format(shp[0],shp[1]))
                    

def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    rclpy.spin(image_node)


if __name__ == '__main__':
    main()