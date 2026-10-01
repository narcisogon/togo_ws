#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSListener(Node):
    def __init__(self):
        super().__init__('gps_listener')
        
        # Subscribe to primary GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/husky/sensors/ins_0/gps_0/data_raw',
            self.gps_callback,
            10
        )
        
        self.gps_count = 0
        self.get_logger().info('GPS Listener started - waiting for GPS data...')
        self.get_logger().info('This demonstrates Step 4: Listen to GPS and print to console')
    
    def gps_callback(self, msg):
        """Print GPS data when received"""
        self.gps_count += 1
        
        # Print every 10 messages to avoid spam
        if self.gps_count % 10 == 0:
            self.get_logger().info(
                f'GPS Fix #{self.gps_count} - '
                f'Lat: {msg.latitude:.8f}, '
                f'Lon: {msg.longitude:.8f}, '
                f'Alt: {msg.altitude:.2f}m, '
                f'Status: {msg.status.status} '
                f'(Frame: {msg.header.frame_id})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = GPSListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GPS Listener shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
