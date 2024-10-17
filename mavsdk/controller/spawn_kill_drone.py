import numpy as np
from geopy.distance import geodesic
import os
import argparse
import signal
import subprocess
import time

# 홈 기본 좌표
home_latitude = 35.8907
home_longitude = 128.6122

# PX4-Autopilot 절대 경로 설정 (환경에 따라 수정)
PX4_AUTOPILOT_DIR = "/home/rkdwhddud/uam-control-system-simulator/PX4-Autopilot"

# 인스턴스 관리하는 txt 파일 경로 (환경에 따라 수정)
INSTANCE_FILE_PATH = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/drone_instances.txt"

ROS2_PRINT_NODE_PATH = "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/ws_ros2_drone"

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
        f"HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='{y_disp},{x_disp}' "
        f"PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i {instance_id}"
    )

    ros2_print_command = ["bash", "-c", f"source install/setup.bash && ros2 run print_position print_position {instance_id}"]
    
    # PX4-Autopilot 디렉토리로 이동
    os.chdir(PX4_AUTOPILOT_DIR)
    
    #print(f"실행할 PX4 명령어: {px4_command}")
    
    # 명령어 실행 및 python PID 저장 (나중에 파이썬 프로세스가 종료될 때 자식들도 같이 종료하기 위함)
    process = subprocess.Popen(px4_command, shell=True, preexec_fn=os.setsid) # 프로세스 그룹으로 묶기
    #time.sleep(10)
    # position_process = subprocess.Popen(
    #     ["python3", "/home/rkdwhddud/uam-control-system-simulator/mavsdk/controller/print_position.py", str(instance_id)],
    #     preexec_fn=os.setsid,
        
    # )
    # os.setpgid(position_process.pid, os.getpgid(process.pid))

    os.chdir(ROS2_PRINT_NODE_PATH)
    # subprocess.Popen(['source', 'install/setup.bash'])
    # print_process = subprocess.Popen(ros2_print_command, preexec_fn=os.setsid)
    print_process = subprocess.Popen(ros2_print_command, preexec_fn=os.setsid)

    pid = os.getpid()  # 파이썬 프로세스 PID 추출
    # instance id와 PID 함께 저장
    with open(INSTANCE_FILE_PATH, "a") as f:
        f.write(f"{instance_id},{pid}\n")
        print(f"instance {instance_id} 작성 완료")

    try:
        # 프로세스가 끝날 때까지 대기
        process.wait()
    except KeyboardInterrupt:
        # CTRL + C 입력 시 프로세스 그룹을 종료
        print(f"프로세스 그룹 종료: {pid}")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        print(f"px4 - {process.pid} 종료")
        # os.killpg(os.getpgid(position_process.pid), signal.SIGINT)
        # print(f"position - {position_process.pid} 종료")
        os.killpg(os.getpgid(print_process.pid), signal.SIGINT)
        print(f"ros2 - {print_process.pid} 종료")
        exit(0)

def kill_px4_process(instance_id): # 프로세스 종료시키는 함수
    
    # PID 파일 경로
    if not os.path.exists(INSTANCE_FILE_PATH):
        print(f"PID 파일이 존재하지 않습니다: {INSTANCE_FILE_PATH}")
        return
    
    with open(INSTANCE_FILE_PATH, "r") as f:
        lines = f.readlines()

    # 해당 인스턴스 ID에 맞는 PID를 찾기
    pid_to_kill = None
    for line in lines:
        instance, pid = line.strip().split(',')
        if int(instance) == instance_id:
            pid_to_kill = int(pid)
            
    
    if pid_to_kill:
        print(f"인스턴스에 해당하는 프로세스 중지: {pid_to_kill}")
        os.kill(pid_to_kill, signal.SIGINT)  # SIGINT는 Ctrl+C와 같은 효과를 줌
        
        # 종료 후 PID 파일에서 해당 인스턴스 정보를 삭제
        with open(INSTANCE_FILE_PATH, "w") as f:
            for line in lines:
                line = line.strip()
                
                # 공백 줄이면 넘어가기
                if not line:
                    continue
                
                instance, pid = line.split(',')
                if int(instance) != instance_id:
                    f.write(line + "\n")

    else:
        print(f"해당 인스턴스가 존재하지 않음: {instance_id}")
        
    exit(0)

if __name__ == "__main__":
    # 명령줄 인수 처리
    parser = argparse.ArgumentParser()
    
    parser.add_argument("instance_id", type=int) # 인스턴스 번호 파라미터 입력
    parser.add_argument("latitude", type=float) # 생성할 위도 파라미터 입력
    parser.add_argument("longitude", type=float) # 생성할 경도 파라미터 입력
    parser.add_argument("--kill", action='store_true') # 명령줄 선언

    args = parser.parse_args()
    
    if args.kill:
        kill_px4_process(args.instance_id)


    # 드론 생성 실행
    else:
        spawn_drone_at_target(args.latitude, args.longitude, args.instance_id)

