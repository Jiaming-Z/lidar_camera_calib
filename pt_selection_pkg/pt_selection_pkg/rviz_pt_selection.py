import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point

### point selection using the topic : clicking points: node subscribe to /clicked_point topic to rviz node, and in rviz "publish point", click to see point, record clicked point

### integrate calibration algorithum here, output R,t
class InteractiveMarkerNode(Node):

    def __init__(self):
        super().__init__('interactive_marker_node')

        


def main(args=None):
    rclpy.init(args=args)

    node = InteractiveMarkerNode()

    node.main_loop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
