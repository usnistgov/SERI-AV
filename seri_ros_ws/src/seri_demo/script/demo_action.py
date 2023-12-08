#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from seri_demo.vehicle_interface import DemoAction

def main(args=None):
    '''
    Main function showing a multi-threaded executor.
    '''
    rclpy.init(args=args)
    demo_action_node = DemoAction("demo_action")
    executor = MultiThreadedExecutor()
    executor.add_node(demo_action_node)
    try:
        demo_action_node.get_logger().info('Beginning demo, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        demo_action_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    demo_action_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
