#!/usr/bin/env python3

import asyncio
import aiohttp
import grpc._channel
from mavsdk import System
import time
import grpc
import sys

INSTANCE_ID = sys.argv[1]
BACKEND_URL = "http://backend-url/api/drone/position"

async def run():
    # Init the drone
    #time.sleep(5)
    try:
        drone = System()
        print("waiting connection...........")
        await drone.connect(system_address=f"udp://:{14540 + int(INSTANCE_ID)}")
        print("connection success!!!!!!!")
        # Start the tasks
        try: 
            asyncio.ensure_future(print_position(drone))

        except grpc._channel._MultiThreadedRendezvous as e:
            print("이벤트 등록 실패, 재시도")
            time.sleep(1)
            asyncio.ensure_future(print_position(drone))

        while True:
            await asyncio.sleep(1)
    
    except grpc._channel._MultiThreadedRendezvous as e:
        print(f"연결 오류 발생: {e}")
        print("연결을 재시도합니다...")
        await asyncio.sleep(1)  # 5초 대기 후 재시도
        await run()  # 재귀적으로 연결 재시도

async def print_position(drone):
    try:
        async for position in drone.telemetry.position():
            #print(position)
            lat = position.latitude_deg
            lon = position.longitude_deg
            alt = position.absolute_altitude_m
            coordinate = {
                "instance_id": INSTANCE_ID,
                "latitude": lat,
                "longitude": lon,
                "altitude": alt
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(f"{BACKEND_URL}", json=coordinate) as resp:
                    if resp.status == 200:
                        print(f"드론 {INSTANCE_ID} 위치 전송 성공")
                    else:
                        print(f"위치 전송 실패: {resp.status}")

            time.sleep(1)
            
    except KeyboardInterrupt:
        print("print_position 종료")


if __name__ == "__main__":
    # Start the main function
    print("Start printing position!!!!!!!!!!!!!!!!!")
    try:
        asyncio.run(run())

    except grpc._channel._MultiThreadedRendezvous as e:
        print(f"연결 오류 발생: {e}")
        print("연결을 재시도합니다...")
        time.sleep(1)  # 5초 대기 후 재시도
        asyncio.run(run())  # 재귀적으로 연결 재시도 
