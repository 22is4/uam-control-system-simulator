import asyncio
import subprocess
import os
import signal
import json
import aiohttp

# spawn_kill_drone.py 및 mission.py의 경로
SPAWN_KILL_DRONE_SCRIPT = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/spawn_kill_drone.py"
MISSION_SCRIPT = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/mission.py"

# 백엔드 URL 설정 (예시)
BACKEND_URL = "http://localhost:5000/api/drone"


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

        elif command_type == 2:  # 미션 수행
            mission_items = command.get("mission_items")
            await run_mission(instance_id, mission_items)

    except Exception as e:
        print(f"명령 처리 중 오류 발생: {str(e)}")


async def spawn_drone(instance_id, latitude, longitude):
    """spawn_kill_drone.py를 통해 드론 인스턴스를 생성"""
    try:
        # 드론을 생성하기 위해 spawn_kill_drone.py를 실행
        process = subprocess.Popen(
            ["python3", SPAWN_KILL_DRONE_SCRIPT, str(instance_id), str(latitude), str(longitude)]
        )
        # process.wait()는 필요하지 않으므로 삭제, 생성 작업을 비동기적으로 계속 처리
        print(f"드론 인스턴스 {instance_id} 생성 요청 완료")

    except Exception as e:
        print(f"드론 생성 중 오류 발생: {str(e)}")


async def kill_drone(instance_id):
    """spawn_kill_drone.py를 통해 드론 인스턴스를 삭제"""
    try:
        # 드론을 삭제하기 위해 spawn_kill_drone.py의 --kill 옵션 실행
        process = subprocess.Popen(
            ["python3", SPAWN_KILL_DRONE_SCRIPT, "--kill", str(instance_id), "0", "0"]
        )
        # process.wait()는 필요하지 않으므로 삭제
        print(f"드론 인스턴스 {instance_id} 삭제 요청 완료")

    except Exception as e:
        print(f"드론 삭제 중 오류 발생: {str(e)}")


async def run_mission(instance_id, mission_items):
    """mission.py를 통해 미션을 수행"""
    try:
        # mission.py는 mission_items과 함께 실행
        process = await asyncio.create_subprocess_exec(
            "python3", MISSION_SCRIPT, json.dumps(mission_items), str(instance_id)
        )
        await process.wait()

        print(f"드론 인스턴스 {instance_id} 미션 수행 완료")

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
            print(f"명령 수신 오류: {e}")


async def main():
    """메인 이벤트 루프"""
    # 명령을 수신하는 비동기 루프 시작
    command_listener = asyncio.create_task(listen_for_commands())

    await command_listener


if __name__ == "__main__":
    asyncio.run(main())
