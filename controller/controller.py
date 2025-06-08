import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from . import server
import asyncio
import threading
from rclpy.executors import ExternalShutdownException

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        
    async def callback_publish(self, data):
        """ゲームパッドのデータをROS2トピックにパブリッシュする"""
        msg = Joy()
        # print(f"Received data: {data}")

        # 修正ポイント：型変換を明示
        msg.axes = [float(a) for a in data["axes"]]
        msg.buttons = [int(b) for b in data["buttons"]]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Joy message - Axes: %s, Buttons: %s' % (msg.axes, msg.buttons))

def ros_spin_safe(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        print("ROSノードがシャットダウンされました。")
    except KeyboardInterrupt:
        print("ROSスレッドでCtrl+Cを検出（通常は来ないはず）")
    except Exception as e:
        print(f"ROSスレッドで予期しないエラー: {e}")

def main(args=None):
    rclpy.init(args=args)
    global controller
    controller = Controller()
    # rosのサブスレッドを開始
    ros_thread = threading.Thread(target=ros_spin_safe, args=(controller,), daemon=True)
    ros_thread.start()
    
    try:
        asyncio.run(server.start_server(controller.callback_publish))  # サーバー実行（ブロッキング）
    except KeyboardInterrupt:
        print("Ctrl+Cを検出。サーバーを終了します。")
    finally:
        # ノードを破棄
        controller.destroy_node()
        
        # rclpyがまだ生きていたらシャットダウン
        if rclpy.ok():
            rclpy.shutdown()