import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log


class RemapperPub(Node):

    def __init__(self):
        super().__init__('log_remapper', enable_rosout=False, namespace='merlin2')
        
        self.declare_parameter('out_topic_name', '/sealfs/all')
        
        self.rosout_sub_ = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10)
        
        self.sealfs_pub_ = self.create_publisher(
            Log,
            self.get_parameter_or('out_topic_name', '/sealfs/all').get_parameter_value().string_value,
            10)
        
        self.merlin_pub_ = self.create_publisher(
            Log,
            '/sealfs/explainability',
            10)
        
        
    def rosout_callback(self, msg: Log):
        self.sealfs_pub_.publish(msg)
        
        if msg.name.startswith('merlin2'):
            self.merlin_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    remapper_node = RemapperPub()

    rclpy.spin(remapper_node)

    remapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()