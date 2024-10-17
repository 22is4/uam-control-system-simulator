from flask import Flask, jsonify, request
import time

app = Flask(__name__)

# 현재 대기 중인 명령 목록 (여기서 controller.py가 명령을 가져감)
commands = []

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

