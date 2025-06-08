# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# from . import server
# import asyncio

# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('controller')
#         self.publisher_ = self.create_publisher(Joy, '/joy', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0
        
#     # def server_init(self):
#     #     self.get_logger().info('Server initialized')
            

#     def timer_callback(self):
#         msg = Joy()
#         msg.axes = []
#         msg.buttons = []
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1
        
#     def publish(self, axes, buttons):
#         """ゲームパッドのデータをROS2トピックにパブリッシュする"""
#         msg = Joy()
#         msg.axes = axes
#         msg.buttons = buttons
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)
#     # start the server
#     asyncio.run(server.main())

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()