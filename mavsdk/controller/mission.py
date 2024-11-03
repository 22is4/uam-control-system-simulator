#!/usr/bin/env python3

import asyncio
import sys
import json
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def run_all_missions(drones_data):
    for drone_data in drones_data:
        # print(f"drone_data: {drone_data}, type: {type(drone_data)}")
        instance_id = drone_data["instance_id"]
        drone = System(port=50051 + instance_id * 2, sysid=instance_id + 1)
        print(f"-- Start waiting {instance_id}")
        await drone.connect(system_address=f"udp://:{14540 + int(instance_id)}")
        print(f"-- Waiting for drone {instance_id} to connect...")
        
        # 드론이 연결될 때까지 대기
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone {instance_id}!!")
                break

        # 미션 업로드 및 시작을 순차적으로 실행
        await upload_and_start_mission(drone, drone_data["mission_items"])
        print(f"드론 {instance_id} 업로드 완료")
        # await asyncio.sleep(1)

async def upload_and_start_mission(drone, mission_items):
    # 기존 미션 중지 및 제거
    await drone.mission.pause_mission()
    await drone.mission.clear_mission()

    mission_plan = MissionPlan([MissionItem(
        item["latitude"], 
        item["longitude"], 
        item["altitude"], 
        item["speed"],
        True,   # is_fly_through
        float('nan'),   # gimbal pitch
        float('nan'),   # gimbal yaw
        MissionItem.CameraAction.NONE,
        float('nan'),   # loiter time
        float('nan'),   # acceptance radius
        float('nan'),   # yaw angle
        float('nan'),   # land precision
        float('nan'),   # action delay
        MissionItem.VehicleAction.NONE
    ) for item in mission_items])

    # 미션 업로드
    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    # 드론의 글로벌 포지션 확인
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # 드론을 무장하고 미션 시작
    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    print("Mission started successfully")

async def cancel_all_tasks():
    """Cancel all currently running asyncio tasks."""
    tasks = [task for task in asyncio.all_tasks() if task is not asyncio.current_task()]
    print(tasks)
    for task in tasks:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    #print(f"sys: {sys.argv[1]}")
    drone_data = json.loads(sys.argv[1])
    #print(f"loaded drone_data: {drone_data}")
    
    asyncio.run(run_all_missions(drone_data))
