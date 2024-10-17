import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import requests
from px4_msgs.msg import VehicleGlobalPosition
import sys
import asyncio
import aiohttp

class CoordinateSender(Node):

    def __init__(self, instance_id):
        super().__init__('coordinate_sender')

        self.instance_id = instance_id
        self.backend_url = "http://211.235.65.244:4999/uam/command/coordinate/update"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic_name = None

        if instance_id == 0:
            topic_name = '/fmu/out/vehicle_global_position'
        else:
            topic_name = f'/px4_{instance_id}/fmu/out/vehicle_global_position'

        print(f"topic_name: {topic_name}")

        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            topic_name,
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print("msg 수신됨")
        coordinate = {
            "instance_id": self.instance_id,
            "latitude": msg.lat,
            "longitude": msg.lon,
            "altitude": msg.alt
        }

        try:
            print(f"backend sending to {self.backend_url}...")
            response = requests.post(self.backend_url, json=coordinate)
            print(f'Sent to backend: {response.status_code}, instance: {coordinate["instance_id"]} lat: {coordinate["latitude"]} lon: {coordinate["longitude"]}')
        except Exception as e:
            print(f"Error sending to backend: {e}")

        

def main():

    instance_id = int(sys.argv[1])

    rclpy.init()
    coordinate_sender = CoordinateSender(instance_id)

    try:
        print(f"print position {instance_id} 실행")
        rclpy.spin(coordinate_sender)
    
    except KeyboardInterrupt:
        coordinate_sender.destroy_node()
        #rclpy.shutdown()
        print(f"{instance_id}번 print_position node 종료")


if __name__ == '__main__':
    main()
