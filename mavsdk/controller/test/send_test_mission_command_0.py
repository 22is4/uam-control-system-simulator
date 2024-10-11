import requests
import json

# 백엔드 URL
BACKEND_URL = "http://localhost:5000/api/send-command"

# 드론 인스턴스 ID 설정
instance_id = 0

# 드론의 home 위치 (위도 35.8907, 경도 128.6122)
home_latitude = 35.8907
home_longitude = 128.6122

# 미션 아이템: home 위치 주변의 경로 설정 (위도, 경도, 고도, 속도)
mission_items = [
    {
        "latitude": home_latitude + 0.001,  # 약간 북쪽으로 이동
        "longitude": home_longitude + 0.001,  # 약간 동쪽으로 이동
        "altitude": 50,  # 고도 20m
        "speed": 10 # 속도 5 m/s
    },
    {
        "latitude": home_latitude - 0.001,  # 약간 남쪽으로 이동
        "longitude": home_longitude - 0.001,  # 약간 서쪽으로 이동
        "altitude": 50,  # 고도 30m
        "speed": 10  # 속도 7 m/s
    },
    {
        "latitude": home_latitude + 0.001,  # 더 북쪽으로 이동
        "longitude": home_longitude - 0.001,  # 서쪽으로 이동
        "altitude": 50,  # 고도 25m
        "speed": 10  # 속도 6 m/s
    }
]

# 명령 생성 (2: 미션 명령)
command = {
    "type": 2,  # 미션 수행 명령
    "instance_id": instance_id,
    "mission_items": mission_items
}

# Flask 서버에 명령 요청 전송
response = requests.post(f"{BACKEND_URL}", json=command)

if response.status_code == 200:
    print(f"Successfully sent mission command to drone {instance_id}.")
else:
    print(f"Failed to send mission command. Status code: {response.status_code}, Response: {response.text}")

