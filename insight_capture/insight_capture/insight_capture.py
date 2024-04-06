import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from insight_capture.insight_native_client import InSightNativeClient

class InSightCapture(Node):

    def __init__(self):
        super().__init__("insight_capture")

        # ROSパラメータの登録
        self.camera_ip = "127.0.0.1"
        self.camera_port = 23
        self.camera_user = "admin"
        self.camera_pw = ""
        self.lib_path = "libinsight_capture_cpp_lib.so"        

        self.declare_parameter("camera_ip", self.camera_ip)
        self.declare_parameter("camera_port", self.camera_port)
        self.declare_parameter("camera_user", self.camera_user)
        self.declare_parameter("camera_pw", self.camera_pw)
        self.declare_parameter("lib_path", self.lib_path)

        self.camera_ip = self.get_parameter("camera_ip").get_parameter_value().string_value
        self.camera_port = self.get_parameter("camera_port").get_parameter_value().integer_value
        self.camera_user = self.get_parameter("camera_user").get_parameter_value().string_value
        self.camera_pw = self.get_parameter("camera_pw").get_parameter_value().string_value
        self.lib_path = self.get_parameter("lib_path").get_parameter_value().string_value        

        # InSight2000カメラクライアントクラス
        self.client = InSightNativeClient(
                self.camera_ip, self.camera_port,
                self.camera_user, self.camera_pw,
                0.5,
                self.lib_path)
        
        # カメラ型変換
        self.bridge = CvBridge()

        # ROSパラメータサーバーの登録
        self.add_on_set_parameters_callback(self.parameters_cb)

        # カメラ画像配信の登録
        self.image_pub = self.create_publisher(Image, "/camera", 10)

        # タイマーコールバックの登録
        self.timer = self.create_timer(1.0, self.timer_cb)

    def parameters_cb(self, params):
        for param in params:
            if param.name == "camera_ip":
                self.camera_ip = param.value
            elif param.name == "camera_port":
                self.camera_port = param.value
            elif param.name == "camera_user":
                self.camera_user = param.value
            elif param.name == "camera_pw":
                self.camera_pw = param.value

    def timer_cb(self):
        if not self.client.is_connected:
            res = self.client.connect()
        else:
            if not self.client.is_login:
                res = self.client.login()
            else:
                image = self.client.capture()
                if image:
                    msg = CvBridge.cv2_to_imgmsg(image)
                    self.image_pub.publish(msg)
                else:
                    self.client.close()