import traceback
import rclpy
from insight_capture.insight_capture import InSightCapture

def main(args=None):
    rclpy.init(args=args)

    node = InSightCapture()

    while rclpy.ok():
        node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()