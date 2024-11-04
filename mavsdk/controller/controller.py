import asyncio
import subprocess
import os
import signal
import json
import aiohttp
import time
import sys
from dotenv import load_dotenv

# spawn_kill_drone.py 및 mission.py의 경로
SPAWN_KILL_DRONE_SCRIPT = None
MISSION_SCRIPT = None
SCENARIO_DIR = None
ROS2_TOPIC_CHECK_NODE_PATH = None


# 백엔드 URL 설정 (예시)
BACKEND_URL = None

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
    
# async def request_backend_approval(drone_data):
#     """백엔드에서 허가 요청"""
#     async with aiohttp.ClientSession() as session:
#         async with session.post(f"{BACKEND_URL}/approve", json=drone_data) as resp:
#             if resp.status == 200:
#                 return await resp.json()  # 승인된 데이터
#             else:
#                 print(f"백엔드 허가 요청 실패: {resp.status}")
#                 return None

async def send_mission_to_backend(drone):
    """Send mission data for each drone instance to the backend."""
    async with aiohttp.ClientSession() as session:
        mission_data = {
            "type": 2,
            "instance_id": drone["instance_id"],
            "mission_items": drone["mission_items"]
        }
        async with session.post(f"{BACKEND_URL}/uam/command/mission", json=mission_data) as response:
            if response.status == 200:
                message = await response.json()
                print(f"{drone['instance_id']}번 드론 미션 데이터 전송 완료: {message.get('message')}")
            else:
                print(f"{drone['instance_id']}번 드론 미션 데이터 전송 실패: {response.status}")


async def listen_for_path_updates(drone_data):
    """ /uam/path에서 특정 드론 인스턴스의 경로 업데이트를 대기하여 run_mission을 호출 """
    async with aiohttp.ClientSession() as session:
        while True:
            async with session.get(f"{BACKEND_URL}/uam/path") as response:
                if response.status == 200:
                    # json 객체를 하나 받아오기
                    update = await response.json()
                    
                    # 업데이트 객체에서 instanceId 추출
                    instance_id = update.get("instanceId")
                    if instance_id is None:
                        print("유효하지 않은 업데이트 데이터")
                        continue
                    
                    # 해당 instanceId와 일치하는 드론 인스턴스 찾기
                    drone_to_update = None
                    for drone in drone_data:
                        if drone["instance_id"] == instance_id:
                            drone_to_update = drone
                            break
                    
                    # 해당 드론 인스턴스가 존재하면 경로 업데이트 수행
                    if drone_to_update:
                        new_mission_items = update.get("coordinates")
                        if new_mission_items:
                            # 드론의 mission_items 업데이트
                            drone_to_update["mission_items"] = [
                                {
                                    "latitude": coord["latitude"],
                                    "longitude": coord["longitude"],
                                    "altitude": coord["altitude"],
                                    "speed": drone_to_update["speed"]
                                }
                                for coord in new_mission_items
                            ]

                            # 업데이트된 드론 인스턴스를 JSON 배열 형식으로 run_mission 수행
                            mission_data = {"drones": [drone_to_update]}
                            # print(f"mission_data: {mission_data}")
                            print(f"{instance_id}번 드론에 대한 업데이트 경로 미션 전달 시작.")
                            await run_mission(mission_data["drones"])
                            print(f"{instance_id}번 드론에 대한 업데이트 경로 미션 전달 완료.")
                            # await asyncio.sleep(3)
                    else:
                        print(f"{instance_id}번 드론을 찾을 수 없음.")
            # await asyncio.sleep(1)

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
        await asyncio.sleep(1)

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

        ros2_check_topic_command = ["bash", "-c", f"source install/setup.bash && ros2 run topic_checker topic_checker {instance_id}"]

        os.chdir(ROS2_TOPIC_CHECK_NODE_PATH)
        # subprocess.Popen(['source', 'install/setup.bash'])
        # print_process = subprocess.Popen(ros2_print_command, preexec_fn=os.setsid)
        # topic_check_process = subprocess.Popen(ros2_check_topic_command)
        topic_check_process = await asyncio.create_subprocess_exec(*ros2_check_topic_command, preexec_fn=os.setsid)

        print(f"{instance_id}번 드론 토픽 확인 대기중.....")
        await topic_check_process.wait()
        print(f"{instance_id}번 드론 토픽 확인 완료")

        if topic_check_process.returncode is None:  # returncode가 None이면 아직 실행 중
            os.killpg(os.getpgid(topic_check_process.pid), signal.SIGTERM)
            

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
    #print(f"run_mission 시작: {drone_data}")
    try:
        # mission.py는 mission_items과 함께 실행
        process = await asyncio.create_subprocess_exec(
            "python3", MISSION_SCRIPT, json.dumps(drone_data)
        )
        # await process.wait()

        print(f"process pid: {process.pid}")

        print(f"드론 인스턴스 미션 전달")

    except Exception as e:
        print(f"미션 수행 오류: {e}")
        process.kill()

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


async def cancel_all_tasks():
    """Cancel all currently running asyncio tasks."""
    tasks = [task for task in asyncio.all_tasks() if task is not asyncio.current_task()]
    # print(tasks)
    for task in tasks:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    print("모든 task 종료")

async def main(scenario_id):
    try:
        scenario_data = await read_scenario_file(scenario_id)

        drone_data = scenario_data['drones']

        # 드론 생성
        await spawn_drones(drone_data)
        # print("드론 생성, 10초 기다리기")
        # time.sleep(10)
        # 백엔드에 미션 전송
        # approval = await request_backend_approval(drone_data)

        # if approval and approval["status"] == "approved":
        #     # mission_override 확인
        #     if "mission_override" in approval:
        #         mission_override = approval["mission_override"]
        #         if mission_override:
        #             for drone in drone_data:
        #                 for override in mission_override:
        #                     if drone["instance_id"] == override["instance_id"]:
        #                         drone["mission_items"] = override["mission_items"]

        #     await run_mission(drone_data)
        #     await listen_for_commands()

        for drone in drone_data:
            await send_mission_to_backend(drone)

        await asyncio.sleep(3)

        await listen_for_path_updates(drone_data)

        # else:
        #     print("미션이 백엔드에서 거부되었습니다. 모든 드론을 종료합니다.")
        #     await kill_drones(drone_data)

    # except KeyboardInterrupt:
    #     await cancel_all_tasks()
    #     print("controller: main 비동기 작업 처리 후 종료")

    except Exception as e:
        print(f"main 함수에서 예외 발생: {e}")
        await kill_drones(drone_data)
        await cancel_all_tasks()
        print("controller 종료")

if __name__ == "__main__":
    scenario_id = sys.argv[1]

    env_path = load_project_env()
    print(f"env_path: {env_path}")
    load_dotenv(env_path)

    SPAWN_KILL_DRONE_SCRIPT = os.path.join(os.getenv("BASE_DIR"), os.getenv("SPAWN_KILL_DRONE_SCRIPT")[1:])
    MISSION_SCRIPT = os.path.join(os.getenv("BASE_DIR"), os.getenv("MISSION_SCRIPT")[1:])
    SCENARIO_DIR = os.path.join(os.getenv("BASE_DIR"), os.getenv("SCENARIO_DIR")[1:])
    ROS2_TOPIC_CHECK_NODE_PATH = os.path.join(os.getenv("BASE_DIR"), os.getenv("ROS2_TOPIC_CHECK_NODE_PATH")[1:])

    BACKEND_URL = os.getenv("BACKEND_URL")

    # print(f"base: {os.getenv('BASE_DIR')}")
    # print(f"spawn: {SPAWN_KILL_DRONE_SCRIPT}")
    # print(f"check: {os.path.join(os.getenv('BASE_DIR'), os.getenv('SCENARIO_DIR'))}")
    # print(f"scenario: {SCENARIO_DIR}")
    try:
        asyncio.run(main(scenario_id))
    except KeyboardInterrupt as e:
        print(f"scenario 종료, ctrl + c를 눌러 controller.py를 종료해 주세요: {e}")
        try:
            while True:
                pass
        except KeyboardInterrupt as e:
            print(f"controller.py 종료")

    except Exception as e:
        print(f"controller.py 종료됨: {e}")
    
