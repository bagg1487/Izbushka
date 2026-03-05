from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
import socket
import struct
import cv2
import numpy as np
import asyncio
import base64
import threading
import subprocess
import platform
import os
from contextlib import asynccontextmanager

robot_socket = None
robot_video_socket = None
robot_connected = False
current_ip = None
video_thread_running = False
last_frame = None
sonic_value = "0"

frame_lock = threading.Lock()
sonic_lock = threading.Lock()

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Server started")
    yield
    global robot_socket, robot_video_socket, robot_connected
    if robot_socket:
        robot_socket.close()
    if robot_video_socket:
        robot_video_socket.close()
    robot_connected = False
    print("Server stopped")

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def serve_index():
    """Отдает index.html при заходе на корневой URL"""
    index_path = os.path.join("app", "static", "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"error": "index.html not found"}

@app.post("/connect")
async def connect(ip: str):
    global robot_socket, robot_video_socket, robot_connected, current_ip, video_thread_running
    
    try:
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.connect((ip, 5001))
        
        robot_video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_video_socket.connect((ip, 8001))
        
        robot_connected = True
        current_ip = ip
        
        if not video_thread_running:
            video_thread = threading.Thread(target=video_receiver, daemon=True)
            video_thread.start()
            video_thread_running = True
        
        with open('IP.txt', 'w') as f:
            f.write(ip)
            
        return {"status": "connected", "ip": ip}
    except Exception as e:
        raise HTTPException(500, f"Connection failed: {e}")

@app.post("/disconnect")
async def disconnect():
    global robot_socket, robot_video_socket, robot_connected
    robot_connected = False
    if robot_socket:
        robot_socket.close()
    if robot_video_socket:
        robot_video_socket.close()
    return {"status": "disconnected"}

@app.get("/status")
async def status():
    return {
        "connected": robot_connected,
        "ip": current_ip if robot_connected else None
    }

@app.get("/api/ssh")
async def get_ssh_info():
    """Возвращает информацию для SSH подключения"""
    system = platform.system()
    try:
        # Получаем IP адрес сервера
        if system == "Windows":
            result = subprocess.run(['ipconfig'], capture_output=True, text=True)
            # Парсим IP из ipconfig
            import re
            ip_match = re.search(r'IPv4-адрес[^\d]*([\d.]+)', result.stdout)
            local_ip = ip_match.group(1) if ip_match else 'unknown'
        else:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            local_ip = result.stdout.strip().split()[0] if result.stdout else 'unknown'
        
        # Получаем имя пользователя
        username = subprocess.run(['whoami'], capture_output=True, text=True).stdout.strip()
        
        return {
            "status": "ok",
            "ssh_command": f"ssh {username}@{local_ip} -p 22",
            "username": username,
            "ip": local_ip,
            "port": 22
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

# Для AnyDesk
@app.get("/api/anydesk")
async def get_anydesk_info():
    """Возвращает информацию для AnyDesk подключения"""
    try:
        system = platform.system()
        
        if system == "Windows":
            # Пытаемся найти AnyDesk и получить ID
            possible_paths = [
                r"C:\Program Files (x86)\AnyDesk\AnyDesk.exe",
                r"C:\Program Files\AnyDesk\AnyDesk.exe"
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    result = subprocess.run([path, '--get-id'], capture_output=True, text=True)
                    anydesk_id = result.stdout.strip()
                    return {
                        "status": "ok",
                        "anydesk_id": anydesk_id,
                        "message": f"AnyDesk ID: {anydesk_id}"
                    }
        else:
            # Linux/Mac
            result = subprocess.run(['anydesk', '--get-id'], capture_output=True, text=True)
            anydesk_id = result.stdout.strip()
            return {
                "status": "ok",
                "anydesk_id": anydesk_id,
                "message": f"AnyDesk ID: {anydesk_id}"
            }
        
        return {"status": "error", "message": "AnyDesk not found"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

def video_receiver():
    global last_frame, robot_video_socket, robot_connected
    
    while robot_connected:
        try:
            stream_bytes = robot_video_socket.recv(4)
            if not stream_bytes or len(stream_bytes) < 4:
                continue
                
            leng = struct.unpack('<L', stream_bytes[:4])[0]
            jpg = robot_video_socket.recv(leng)
            
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                with frame_lock:
                    last_frame = frame
        except:
            break

def send_command(cmd: str):
    global robot_socket, robot_connected
    if robot_connected and robot_socket:
        try:
            robot_socket.send(cmd.encode('utf-8'))
            return True
        except:
            robot_connected = False
    return False

@app.websocket("/ws/video")
async def video_stream(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if robot_connected and last_frame is not None:
                with frame_lock:
                    frame = last_frame.copy()
                
                _, buffer = cv2.imencode('.jpg', frame)
                frame_base64 = base64.b64encode(buffer).decode('utf-8')
                
                await websocket.send_json({
                    "type": "video",
                    "data": frame_base64,
                    "sonic": sonic_value
                })
            
            await asyncio.sleep(0.03)
    except:
        pass

@app.post("/api/commands/move/{direction}")
async def move(direction: str, speed: int = 50):
    if not robot_connected:
        raise HTTPException(400, "Robot not connected")
    
    commands = {
        "forward": "CMD_MOVE_FORWARD",
        "backward": "CMD_MOVE_BACKWARD",
        "left": "CMD_MOVE_LEFT",
        "right": "CMD_MOVE_RIGHT",
        "stop": "CMD_MOVE_STOP"
    }
    
    if direction in commands:
        send_command(f"{commands[direction]}#{speed}\n")
        return {"status": "ok"}
    
    raise HTTPException(400, "Invalid direction")

@app.post("/api/commands/hands")
async def hands_up():
    if robot_connected:
        send_command("CMD_HANDS_UP\n")
    return {"status": "ok"}

@app.post("/api/commands/music/play")
async def music_play():
    if robot_connected:
        send_command("CMD_MUSIC_PLAY\n")
    return {"status": "ok"}

@app.post("/api/commands/music/stop")
async def music_stop():
    if robot_connected:
        send_command("CMD_MUSIC_STOP\n")
    return {"status": "ok"}

@app.post("/api/commands/radio/play")
async def radio_play():
    if robot_connected:
        send_command("CMD_RADIO_PLAY\n")
    return {"status": "ok"}

@app.post("/api/commands/radio/stop")
async def radio_stop():
    if robot_connected:
        send_command("CMD_RADIO_STOP\n")
    return {"status": "ok"}

@app.post("/api/commands/function/{num}")
async def function(num: int):
    if not robot_connected:
        raise HTTPException(400, "Robot not connected")
    
    func_map = {
        1: "CMD_CLENCH_LEFT",
        2: "CMD_CLENCH_LEFT",
        3: "CMD_LOOK_UP",
        4: "CMD_LOOK_DOWN"
    }
    
    if num in func_map:
        send_command(f"{func_map[num]}\n")
        return {"status": "ok"}
    
    raise HTTPException(400, "Invalid function")

@app.post("/api/commands/look/stop")
async def look_stop():
    if robot_connected:
        send_command("CMD_LOOK_STOP\n")
    return {"status": "ok"}

@app.post("/api/commands/sonic")
async def sonic():
    if robot_connected:
        send_command("CMD_SONIC\n")
    return {"status": "ok"}

@app.post("/api/commands/video/toggle")
async def video_toggle(state: bool):
    if robot_connected:
        val = "1" if state else "0"
        send_command(f"CMD_VIDEO_TRANSMISSION#{val}\n")
    return {"status": "ok"}

@app.post("/api/commands/menu")
async def menu():
    if robot_connected:
        send_command("CMD_MENU\n")
    return {"status": "ok"}

@app.post("/api/commands/about/{num}")
async def about(num: int):
    if robot_connected:
        send_command(f"CMD_FUNC_ABOUT#{num}\n")
    return {"status": "ok"}

@app.post("/api/calibration/mode/{state}")
async def calibration_mode(state: bool):
    if robot_connected:
        val = "1" if state else "0"
        send_command(f"CMD_CALIBRATION_MOD#{val}\n")
    return {"status": "ok"}

@app.post("/api/calibration/set")
async def set_calibration(leg: str, servo: str, value: int):
    if robot_connected:
        send_command(f"CMD_CALIBRATION#{leg}#{servo}#{value}\n")
    return {"status": "ok"}

@app.get("/api/calibration/data")
async def get_calibration():
    return {"status": "ok"}  

@app.post("/api/users/add")
async def add_user(name: str):
    if robot_connected:
        send_command("CMD_ADD_USER#1\n")
    return {"status": "ok"}

@app.post("/api/users/photo")
async def take_photo(name: str):
    if robot_connected and name:
        send_command(f"CMD_TAKE_PHOTO#{name}\n")
    return {"status": "ok"}