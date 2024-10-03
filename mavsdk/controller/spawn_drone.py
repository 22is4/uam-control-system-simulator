import numpy as np
from geopy.distance import geodesic
import os
import argparse

# 홈 기본 좌표
home_latitude = 35.8907
home_longitude = 128.6122

# PX4-Autopilot 절대 경로 설정 (환경에 따라 수정)
PX4_AUTOPILOT_DIR = "/home/rkdwhddud/uam-control-system-simulator/PX4-Autopilot"

def calculate_model_pose(home, target):# x, y의 좌표 계산해서 반환하는 함수
    
    # 위도 차이로 x 변위 계산, y 좌표는 동일하게 함
    x_displacement = geodesic((home[0], home[1]), (target[0], home[1])).meters
    if target[0] < home[0]:  # 남쪽으로 이동하는 경우 음수
        x_displacement = -x_displacement
        
    # 경도 차이로 y 변위 계산, x 좌표는 동일하게 함
    y_displacement = geodesic((home[0], home[1]), (home[0], target[1])).meters
    if target[1] < home[1]:  # 서쪽으로 이동하는 경우 음수
        y_displacement = -y_displacement

    #print(f"x_pose: {x_displacement}, y_pose: {y_displacement}")

    return x_displacement, y_displacement

def spawn_drone_at_target(target_latitude, target_longitude, instance_id): # px4 명령어 실행
    
    # 홈 좌표와 목표 좌표 간 변위 계산
    target_coordinates = (target_latitude, target_longitude)
    home_coordinates = (home_latitude, home_longitude)
    
    x_disp, y_disp = calculate_model_pose(home_coordinates, target_coordinates)
    
    # PX4 명령어 작성
    # 왜인진 모르겠지만, y와 x의 자리가 바뀌어야 제대로 된 좌표에 드론이 생성됨
    px4_command = (
        f"PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='{y_disp},{x_disp}' "
        f"PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i {instance_id}"
    )
    
    # PX4-Autopilot 디렉토리로 이동
    os.chdir(PX4_AUTOPILOT_DIR)
    
    #print(f"실행할 PX4 명령어: {px4_command}")
    
    # 명령어 실행
    os.system(px4_command)

if __name__ == "__main__":
    # 명령줄 인수 처리
    parser = argparse.ArgumentParser()
    
    parser.add_argument("latitude", type=float) # 생성할 위도 파라미터 입력
    parser.add_argument("longitude", type=float) # 생성할 경도 파라미터 입력
    parser.add_argument("instance_id", type=int) # 인스턴스 번호 파라미터 입력
    
    args = parser.parse_args()
    
    # 드론 생성 실행
    spawn_drone_at_target(args.latitude, args.longitude, args.instance_id)

