import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import requests
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition, VehicleLandDetected 
import sys
import asyncio
import aiohttp
import math

class CoordinateSender(Node):

    def __init__(self, instance_id):
        super().__init__('coordinate_sender')

        self.instance_id = instance_id
        self.backend_url = "http://localhost:5000/uam/command/update"


        self.global_data = None
        self.local_data = None
        self.land_data = None


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic_global = None
        topic_local = None
        topic_land = None

        if instance_id == 0:
            topic_global = '/fmu/out/vehicle_global_position'
            topic_local = '/fmu/out/vehicle_local_position'
            topic_land = '/fmu/out/vehicle_land_detected'
        else:
            topic_global = f'/px4_{instance_id}/fmu/out/vehicle_global_position'
            topic_local = f'/px4_{instance_id}/fmu/out/vehicle_local_position'
            topic_land = f'/px4_{instance_id}/fmu/out/vehicle_land_detected'

        print(f"topic_global: {topic_global}")
        print(f"topic_local: {topic_local}")
        print(f"topic_land: {topic_land}")

        self.subscription_global = self.create_subscription(
            VehicleGlobalPosition,
            topic_global,
            self.listener_global_callback,
            qos_profile)
        self.subscription_global  # prevent unused variable warning

        self.subscription_local = self.create_subscription(
            VehicleLocalPosition,
            topic_local,
            self.listener_local_callback,
            qos_profile)
        self.subscription_local  # prevent unused variable warning

        self.subscription_land = self.create_subscription(
            VehicleLandDetected,
            topic_land,
            self.listener_land_callback,
            qos_profile)
        self.subscription_land  # prevent unused variable warning

    def listener_global_callback(self, msg):
        if self.global_data == None:
            self.global_data = {
                "lat": msg.lat,
                "lon": msg.lon,
                "alt": msg.alt
            }
        self.send_data()

    def listener_local_callback(self, msg):
        if self.local_data == None:
            self.local_data = {
                "relative_alt": msg.z,
                "vx":msg.vx,
                "vy":msg.vy,
                "vz":msg.vz,
                "speed": math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)
            }
        self.send_data()

    def listener_land_callback(self, msg):
        if self.land_data == None:
            status = None
            if msg.landed:
                status = "landed"
            else:
                status = "flying"
            
            self.land_data = {
                "status": status
            }
        self.send_data()
    
    def send_data(self):
        if self.global_data and self.local_data and self.land_data:
            data = {
                "instance_id": self.instance_id,
                "latitude": self.global_data["lat"],
                "longitude": self.global_data["lon"],
                "altitude": self.global_data["lat"],
                "vx": self.local_data["vx"],
                "vy": self.local_data["vy"],
                "vz": self.local_data["vz"],
                "speed": self.local_data["speed"],
                "status": self.land_data["status"]
            }

            self.global_data = None
            self.local_data = None
            # self.land_data = None

            try:
                rep = requests.post(self.backend_url, json=data)
                # print(f"{self.instance_id}번 드론 update post 완료")
            
            except Exception as e:
                print(f"{self.instance_id}번 update error: {e}")

        # else:
        #     if self.global_data == None:
        #         print(f"{self.instance_id}번 global data is None")
        #     if self.local_data == None:
        #         print(f"{self.instance_id}번 local data is None")
        #     if self.land_data == None:
        #         print(f"{self.instance_id}번 land data is None")
            


        

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
