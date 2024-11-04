from flask import Flask, jsonify, request
import time
import requests

app = Flask(__name__)

# 현재 대기 중인 명령 목록 (여기서 controller.py가 명령을 가져감)
commands = []

path_updates = []

@app.route('/uam/command/mission', methods=['POST'])
def receive_mission():
    """ 드론 미션을 수신하고 /uam/path에 PUT 요청으로 게시 """
    mission_data = request.json
    #print(f"mission_data: {mission_data}")
    instance_id = mission_data.get("instance_id")
    mission_items = mission_data.get("mission_items")

    if instance_id is not None and mission_items is not None:
        path_data = {
            "instanceId": instance_id,
            "coordinates": [
                {
                    "latitude": item["latitude"],
                    "longitude": item["longitude"],
                    "altitude": item["altitude"]
                }
                for item in mission_items
            ]
        }

        #print(f"path_data: {path_data}")

        # /uam/path에 PUT 요청
        try:
            response = requests.put("http://localhost:5000/uam/path", json=path_data)
            #print(f"{instance_id}번 드론의 미션 경로 게시 완료: {response.json().get('message')}, {response.json().get('instance_id')}, {response.json().get('coordinates')}")
            return jsonify({"message": "미션 데이터 수신 및 경로 게시 완료"}), 200
        except requests.exceptions.RequestException as e:
            print(f"경로 게시 오류: {e}")
            return jsonify({"message": "경로 게시 오류"}), 500
    else:
        return jsonify({"message": "유효하지 않은 미션 데이터"}), 400
    
@app.route('/uam/path', methods=['PUT'])
def update_path():
    """/uam/path에 PUT 요청으로 받은 경로 업데이트 저장"""
    update_data = request.json
    print(update_data)
    instance_id = update_data.get("instanceId")
    coord = update_data.get("coordinates")

    if instance_id != None and "coordinates" in update_data:
        # 경로 업데이트를 저장
        path_updates.append(update_data)
        print(f"{instance_id}번 드론의 경로 업데이트 저장 완료.")
        return jsonify({"message": "경로 업데이트 저장 완료"}), 200
    else:
        return jsonify({"message": "유효하지 않은 경로 업데이트 데이터", "instance_id":instance_id, "coordinates": coord}), 400
    
@app.route('/uam/path', methods=['GET'])
def get_path_updates():
    """/uam/path에서 대기 중인 경로 업데이트를 전송"""
    global path_updates
    if len(path_updates) != 0:
        # 대기 중인 경로 업데이트가 있을 경우, 첫 번째 데이터를 반환하고 리스트에서 제거
        next_update = path_updates.pop(0)
        return jsonify(next_update), 200
    else:
        return jsonify({"message": "대기 중인 경로 업데이트 없음"}), 204


@app.route('/uam/command/update', methods=['POST'])
def update_drone_data():
    """Receive and process drone status updates from ROS2 node"""
    data = request.json
    instance_id = data.get("instance_id")
    latitude = data.get("latitude")
    longitude = data.get("longitude")
    altitude = data.get("altitude")
    vx = data.get("vx")
    vy = data.get("vy")
    vz = data.get("vz")
    speed = data.get("speed")
    status = data.get("status")

    # Log received data and send a success response
    # print(f"Received update for drone {instance_id}:")
    # print(f"  Position: ({latitude}, {longitude}, {altitude})")
    # print(f"  Velocity: vx={vx}, vy={vy}, vz={vz}, speed={speed}")
    # print(f"  Status: {status}")
    
    return jsonify({"message": f"Drone {instance_id} data updated successfully"}), 200

@app.route('/uam/command/create', methods=['POST'])
def create_drone():
    data = request.json
    instance_id = data.get("instance_id")
    if instance_id is not None:
        print(f"드론 생성 요청: instance_id {instance_id}")
        return jsonify({"message": "드론 생성 완료"}), 200
    return jsonify({"message": "유효하지 않은 드론 생성 요청입니다."}), 400

@app.route('/uam/command/delete', methods=['DELETE'])
def delete_drone():
    data = request.json
    instance_id = data.get("instance_id")
    if instance_id is not None:
        print(f"드론 삭제 요청: instance_id {instance_id}")
        return jsonify({"message": "드론 삭제 완료"}), 200
    return jsonify({"message": "유효하지 않은 드론 삭제 요청입니다."}), 400

@app.route('/api/drone/commands', methods=['GET'])
def get_commands():
    global commands
    # controller.py가 GET 요청을 보내면, 명령을 반환하고, 명령 목록 초기화
    response = jsonify(commands)
    commands = []  # 명령 처리 후 리스트를 비움
    return response, 200

@app.route('/api/send-command', methods=['POST'])
def send_command():
    """드론 명령을 추가"""
    global commands
    data = request.json
    commands.append(data)
    return jsonify({"status": "success"}), 200

@app.route('/api/drone/approve', methods=['POST'])
def approve_scenario():
    """시나리오 ID에 대한 승인 요청을 처리"""
    global approval_responses
    data = request.json
    #scenario_id = data.get("scenario_id")
    print("10초 뒤에 승인")
    time.sleep(10)
    # 여기에서 시나리오 ID 기반으로 처리 로직을 추가할 수 있음
    if data:
        # 예시로 모든 시나리오를 승인하는 로직
        resp = {
            "status": "approved",
            "mission_override": None  # 필요시 미션을 덮어쓸 수 있음
        }
        return jsonify(resp), 200
    else:
        return jsonify({"status": "error", "message": "Scenario ID not found"}), 400

if __name__ == '__main__':
    # Flask 서버 실행
    app.run(host='0.0.0.0', port=5000)

