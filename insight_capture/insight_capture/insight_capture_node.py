import traceback
import rclpy
from insight_capture.insight_capture import InSightCapture


def main(args=None):
    rclpy.init(args=args)
    node = InSightCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()