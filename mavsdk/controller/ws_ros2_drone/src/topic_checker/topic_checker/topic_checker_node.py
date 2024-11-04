# topic_check_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import requests
from px4_msgs.msg import VehicleGlobalPosition
import sys
import asyncio
import aiohttp
import math
import os
import signal

class TopicCheckNode(Node):
    def __init__(self, instance_id):
        super().__init__('topic_check_node')
        self.instance_id = instance_id

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic_global = None

        if instance_id == 0:
            topic_global = '/fmu/out/vehicle_global_position'
            
        else:
            topic_global = f'/px4_{instance_id}/fmu/out/vehicle_global_position'

        self.subscription_global = self.create_subscription(
            VehicleGlobalPosition,
            topic_global,
            self.listener_global_callback,
            qos_profile)
        self.subscription_global  # prevent unused variable warning

    def listener_global_callback(self, msg):
        print(f"{self.instance_id}번 topic 확인")
        # rclpy.shutdown()
        os.kill(os.getpid(), signal.SIGTERM)
            

def main(args=None):
    instance_id = int(sys.argv[1])

    rclpy.init()
    topic_checker = TopicCheckNode(instance_id)

    try:
        print(f"topic checker for {instance_id}...")
        rclpy.spin(topic_checker)
    
    except Exception as e:
        topic_checker.destroy_node()
        # rclpy.shutdown()
        print(f"{instance_id}번 topic checker 종료")

    #rclpy.shutdown()

if __name__ == '__main__':
    main()
