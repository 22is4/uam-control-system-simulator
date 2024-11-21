import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import requests
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus
import sys
import asyncio
import aiohttp
import math
import os
from dotenv import load_dotenv

def load_project_env():
    # 현재 파일의 절대 경로를 얻어냄
    current_file_path = os.path.abspath(__file__)
    
    # 최상위 디렉토리 'uam-control-system-simulator'를 찾음
    project_root = "uam-control-system-simulator"
    project_root_index = current_file_path.find(project_root)
    
    if project_root_index == -1:
        raise FileNotFoundError(f"'{project_root}' 디렉토리를 찾을 수 없습니다.")

    # 최상위 디렉토리의 경로를 계산
    project_root_path = current_file_path[:project_root_index + len(project_root)]
    env_path = os.path.join(project_root_path, ".env")
    return env_path

class CoordinateSender(Node):

    def __init__(self, instance_id):
        super().__init__('coordinate_sender')

        env_path = load_project_env()
        load_dotenv(env_path)

        self.instance_id = instance_id
        self.backend_url = os.path.join(os.getenv("BACKEND_URL"), "uam/command/update")


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
            topic_land = '/fmu/out/vehicle_status'
        else:
            topic_global = f'/px4_{instance_id}/fmu/out/vehicle_global_position'
            topic_local = f'/px4_{instance_id}/fmu/out/vehicle_local_position'
            topic_land = f'/px4_{instance_id}/fmu/out/vehicle_status'

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
            VehicleStatus,
            topic_land,
            self.listener_land_callback,
            qos_profile)
        self.subscription_land  # prevent unused variable warning

        self.timer_period = 0.3
        self.timer = self.create_timer(self.timer_period, self.send_data)


    def listener_global_callback(self, msg):
        if self.global_data == None:
            self.global_data = {
                "lat": msg.lat,
                "lon": msg.lon,
                "alt": msg.alt
            }
        # self.send_data()

    def listener_local_callback(self, msg):
        if self.local_data == None:
            self.local_data = {
                # "alt": msg.z,
                "relative_alt": msg.z,
                "vx":msg.vx,
                "vy":msg.vy,
                "vz":msg.vz,
                "speed": math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)
            }
        # self.send_data()

    def listener_land_callback(self, msg):
        # print('vehiche_status msg 받음')
        
        status = None
        if msg.nav_state in [0, 18, 12, 20]:
            status = "land"
        elif msg.nav_state == 3:  # AUTO_MISSION
            status = "mission mode"
        elif msg.nav_state == 17:
            status = "takeoff"
        else:
            status = "holding position"
        
        self.land_data = {
            "status": status
        }

        # print(f"nav_state: {msg.nav_state}")
        # self.send_data()
    
    def send_data(self):
        # print(f'{self.instance_id}번 위치 update 전송 시도')
        if self.global_data and self.local_data and self.land_data:
            data = {
                "instanceId": self.instance_id,
                "latitude": self.global_data["lat"],
                "longitude": self.global_data["lon"],
                "altitude": self.global_data["alt"],
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
                # print(f'{self.instance_id}번 위치 update 전송 완료')
                # print(f"{self.instance_id}번 드론 속도 {data['speed']}")
                # print(f"{self.instance_id}번 드론 고도 {data['altitude']}")
            
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
    # print(f"현재 경로: {os.path.abspath(__file__)}")

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
