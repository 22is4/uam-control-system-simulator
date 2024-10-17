import asyncio
import subprocess
import os
import signal
import json
import aiohttp
import time
import sys

# spawn_kill_drone.py 및 mission.py의 경로
SPAWN_KILL_DRONE_SCRIPT = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/spawn_kill_drone.py"
MISSION_SCRIPT = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/mission.py"
SCENARIO_DIR = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/scenario"

# 백엔드 URL 설정 (예시)
BACKEND_URL = "http://localhost:5000/api/drone"

async def read_scenario_file(scenario_id):
    """Read scenario file and return drone info."""
    scenario_number = f"scenario_{scenario_id}.json"
    file_path = os.path.join(SCENARIO_DIR, scenario_number)
    print(file_path)
    try:
        with open(file_path, "r") as f:
            drone_data = json.load(f)
        return drone_data
    except Exception as e:
        print(f"Error reading scenario file: {e}")
        return []
    
async def request_backend_approval(drone_data):
    """백엔드에서 허가 요청"""
    async with aiohttp.ClientSession() as session:
        async with session.post(f"{BACKEND_URL}/approve", json=drone_data) as resp:
            if resp.status == 200:
                return await resp.json()  # 승인된 데이터
            else:
                print(f"백엔드 허가 요청 실패: {resp.status}")
                return None

async def handle_command(command):
    """비동기적으로 명령을 처리 (생성, 삭제, 미션)"""
    try:
        command_type = command.get("type")  # 0: 생성, 1: 삭제, 2: 미션
        instance_id = command.get("instance_id")

        if command_type == 0:  # 드론 생성
            latitude = command.get("latitude")
            longitude = command.get("longitude")
            await spawn_drone(instance_id, latitude, longitude)

        elif command_type == 1:  # 드론 삭제
            await kill_drone(instance_id)

        # elif command_type == 2:  # 미션 수행
        #     drone_data = command.get("drones")
        #     await run_mission(drone_data)

    except Exception as e:
        print(f"명령 처리 중 오류 발생: {str(e)}")


async def spawn_drones(drone_data):
    """드론 인스턴스 생성"""
    for drone in drone_data:
        instance_id = drone["instance_id"]
        latitude = drone["latitude"]
        longitude = drone["longitude"]
        await spawn_drone(instance_id, latitude, longitude)

async def spawn_drone(instance_id, latitude, longitude):
    """spawn_kill_drone.py를 통해 드론 인스턴스를 생성"""
    try:
        # 드론을 생성하기 위해 spawn_kill_drone.py를 실행
        process = await asyncio.create_subprocess_exec(
            "python3", SPAWN_KILL_DRONE_SCRIPT, str(instance_id), str(latitude), str(longitude)
        )
        print(f"controller - {os.getpid()}")
        print(f"control_spawn - {process.pid}")
        # process.wait()는 필요하지 않으므로 삭제, 생성 작업을 비동기적으로 계속 처리
        print(f"드론 인스턴스 {instance_id} 생성 요청 완료")

        # await asyncio.sleep(1)
        # process.wait()

    except Exception as e:
        print(f"드론 생성 중 오류 발생: {str(e)}")

async def kill_drones(drone_data):
    """모든 드론 인스턴스를 종료"""
    for drone in drone_data:
        await kill_drone(drone["instance_id"])

async def kill_drone(instance_id):
    """spawn_kill_drone.py를 통해 드론 인스턴스를 삭제"""
    try:
        # 드론을 삭제하기 위해 spawn_kill_drone.py의 --kill 옵션 실행
        process = await asyncio.create_subprocess_exec(
            "python3", SPAWN_KILL_DRONE_SCRIPT, "--kill", str(instance_id), "0", "0"
        )
        print(f"controller - {os.getpid()}")
        print(f"control_kill - {process.pid}")
        # process.wait()는 필요하지 않으므로 삭제
        print(f"드론 인스턴스 {instance_id} 삭제 요청 완료")

        # await asyncio.sleep(1)
        # process.wait()

    except Exception as e:
        print(f"드론 삭제 중 오류 발생: {str(e)}")


async def run_mission(drone_data):
    """mission.py를 통해 미션을 수행"""
    try:
        # mission.py는 mission_items과 함께 실행
        process = await asyncio.create_subprocess_exec(
            "python3", MISSION_SCRIPT, json.dumps(drone_data)
        )
        # await process.wait()

        print(f"드론 인스턴스 미션 전달")

    except Exception as e:
        print(f"미션 수행 오류: {e}")

async def listen_for_commands():
    """명령을 수신하여 처리하는 루프"""
    while True:
        try:
            # 백엔드로부터 명령을 받아옴 (예시: HTTP 통신)
            async with aiohttp.ClientSession() as session:
                async with session.get(f"{BACKEND_URL}/commands") as resp:
                    if resp.status == 200:
                        commands = await resp.json()
                        for command in commands:
                            asyncio.create_task(handle_command(command))

            await asyncio.sleep(1)  # 1초 대기 후 다시 명령 확인

        except Exception as e:
            print(f"명령 대기 중...")
            time.sleep(1)


async def main(scenario_id):
    
    scenario_data = await read_scenario_file(scenario_id)

    drone_data = scenario_data['drones']

    # 드론 생성
    await spawn_drones(drone_data)
    print("드론 생성, 10초 기다리기")
    time.sleep(10)
    # 백엔드에 미션 전송
    approval = await request_backend_approval(drone_data)

    if approval and approval["status"] == "approved":
        # mission_override 확인
        if "mission_override" in approval:
            mission_override = approval["mission_override"]
            if mission_override:
                for drone in drone_data:
                    for override in mission_override:
                        if drone["instance_id"] == override["instance_id"]:
                            drone["mission_items"] = override["mission_items"]

        await run_mission(drone_data)
        await listen_for_commands()
    else:
        print("미션이 백엔드에서 거부되었습니다. 모든 드론을 종료합니다.")
        await kill_drones(drone_data)

if __name__ == "__main__":
    scenario_id = sys.argv[1]
    asyncio.run(main(scenario_id))
