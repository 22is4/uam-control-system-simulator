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

if __name__ == '__main__':
    # Flask 서버 실행
    app.run(host='0.0.0.0', port=5000)

