#!/usr/bin/env python3

import asyncio
import aiohttp
import grpc._channel
from mavsdk import System
import time
import grpc
import sys

INSTANCE_ID = sys.argv[1]
BACKEND_URL = "http://localhost:5000/api/drone/position"

async def run():
    # Init the drone
    #time.sleep(5)
    
    drone = System()
    print("waiting connection...........")
    await drone.connect(system_address=f"udp://:{14540 + int(INSTANCE_ID)}")
    print(f"connected with {14540 + int(INSTANCE_ID)}")
    # print("Waiting for drone to connect...")
    # async for state in drone.core.connection_state():
    #     if state.is_connected:
    #         print(f"-- Connected to drone {14540 + int(INSTANCE_ID)}")
    #         break
        
    asyncio.ensure_future(print_position(drone))

        

    while True:
        await asyncio.sleep(1)

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
                        print(f"드론 {INSTANCE_ID} 위치 전송 실패: {resp.status}")

            await asyncio.sleep(1)
            
    except KeyboardInterrupt:
        print("print_position 종료")
        await shutdown()

async def shutdown():
    # 모든 asyncio task를 취소하고 루프를 종료하는 함수
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]

    for task in tasks:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    await asyncio.get_event_loop().shutdown_asyncgens()
    print("모든 print_position task 종료")
    exit(0)

if __name__ == "__main__":
    # Start the main function
    print("Start printing position!!!!!!!!!!!!!!!!!")
    try:
        asyncio.run(run())

    except KeyboardInterrupt :
        asyncio.run(shutdown())
