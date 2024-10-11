import requests
import time

# Flask 서버 엔드포인트
BACKEND_URL = "http://localhost:5000/api/send-command"

def send_create_drone_command(instance_id, latitude, longitude):
    """드론 생성 명령을 Flask 서버로 전송"""
    command = {
        "type": 0,  # 드론 생성 명령
        "instance_id": instance_id,
        "latitude": latitude,
        "longitude": longitude
    }
    response = requests.post(BACKEND_URL, json=command)
    if response.status_code == 200:
        print(f"드론 생성 명령 전송 성공 (인스턴스 {instance_id})")
    else:
        print(f"드론 생성 명령 전송 실패: {response.status_code}")

def send_delete_drone_command(instance_id):
    """드론 삭제 명령을 Flask 서버로 전송"""
    command = {
        "type": 1,  # 드론 삭제 명령
        "instance_id": instance_id
    }
    response = requests.post(BACKEND_URL, json=command)
    if response.status_code == 200:
        print(f"드론 삭제 명령 전송 성공 (인스턴스 {instance_id})")
    else:
        print(f"드론 삭제 명령 전송 실패: {response.status_code}")

if __name__ == "__main__":
    # 드론 생성 테스트 (instance_id 0)
    send_create_drone_command(0, 35.8907, 128.6122)
    # time.sleep(15)  # 5초 대기

    # 드론 삭제 테스트 (instance_id 0)
    # send_delete_drone_command(0)

